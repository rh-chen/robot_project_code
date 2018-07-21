#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "DBoW3/DBoW3.h"
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/matx.hpp>
#include <sys/stat.h>

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/distortion_models.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <boost/thread/thread.hpp>
#include <boost/make_shared.hpp>
#include <stdio.h>
#include <math.h>
#include <algorithm> 
#include <tf/tf.h>
#include <string>
#include <vector>
#include "relocalization_sweeper/GetRobotPose.h"

#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include "g2o/solvers/csparse/linear_solver_csparse.h"
#include <chrono>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <sophus/se3.h>
#include <sophus/so3.h>

using Sophus::SO3;
using Sophus::SE3;
using Eigen::Vector2d;
using Eigen::Vector3d;
using namespace g2o;
using namespace cv;
using namespace std;

typedef g2o::BlockSolver<g2o::BlockSolverTraits<6,2>> Block;

std::string voc_dir;
std::string dataset_dir;
std::string pose_dir;
std::string depth_dir;
double fx,fy,cx,cy;

namespace relocalization_robot{
using namespace __gnu_cxx;
using namespace std;
using namespace cv;

void match_features_knn(Mat& query, Mat& train, vector<DMatch>& matches)
{
    flann::Index flannIndex(query,flann::LshIndexParams(5,10,2),cvflann::FLANN_DIST_HAMMING);
    Mat matchindex(train.rows,2,CV_32SC1);
    Mat matchdistance(train.rows, 2, CV_32FC1);
    flannIndex.knnSearch(train, matchindex, matchdistance,2,flann::SearchParams());
     
    for (int i = 0; i < matchdistance.rows; i++)
    {
        if (matchdistance.at<float>(i, 0) < 0.6*matchdistance.at<float>(i, 1))
         {
            DMatch dmatches(matchindex.at<int>(i, 0),i, matchdistance.at<float>(i, 0));
            matches.push_back(dmatches);
         }
    }
}

void refine_match_with_homography(vector<KeyPoint>& queryKeyPoints,
                                  vector<KeyPoint>& trainKeyPoints,
                                  float reprojectionThreshold,
                                  vector<DMatch>& matches,
                                  Mat& homography)
{
    std::vector<cv::Point2f> srcPoints(matches.size());
    std::vector<cv::Point2f> dstPoints(matches.size());
    for (size_t i = 0; i < matches.size(); i++)
    {
       srcPoints[i] = trainKeyPoints[matches[i].trainIdx].pt;
       dstPoints[i] = queryKeyPoints[matches[i].queryIdx].pt;
    }

    std::vector<unsigned char> inliersMask(srcPoints.size());
    homography = cv::findHomography(srcPoints,dstPoints,CV_FM_RANSAC,reprojectionThreshold,inliersMask);
    std::vector<cv::DMatch> inliers;
    for (size_t i = 0; i<inliersMask.size(); i++)
    {
        if (inliersMask[i])
           inliers.push_back(matches[i]);
    }
    matches.swap(inliers);
}
void bundleAdjustment (
    const vector<Point3f> points_3d,
    const vector<Point2f> points_2d,
    const Mat& K,
    Mat& R,
    Mat& t,
    Mat& inliers){
    typedef g2o::BlockSolver< g2o::BlockSolverTraits<6,3> > Block;  
    Block::LinearSolverType* linearSolver = new g2o::LinearSolverCSparse<Block::PoseMatrixType>(); 
    Block* solver_ptr = new Block ( linearSolver );  
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg ( solver_ptr );
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm ( solver );

    // vertex
    g2o::VertexSE3Expmap* pose = new g2o::VertexSE3Expmap(); // camera pose
    Eigen::Matrix3d R_mat;
    R_mat <<
          R.at<double> ( 0,0 ), R.at<double> ( 0,1 ), R.at<double> ( 0,2 ),
               R.at<double> ( 1,0 ), R.at<double> ( 1,1 ), R.at<double> ( 1,2 ),
               R.at<double> ( 2,0 ), R.at<double> ( 2,1 ), R.at<double> ( 2,2 );
    pose->setId ( 0 );
    pose->setEstimate ( g2o::SE3Quat (
                            R_mat,
                            Eigen::Vector3d ( t.at<double> ( 0,0 ), t.at<double> ( 1,0 ), t.at<double> ( 2,0 ) )
                        ) );
    optimizer.addVertex ( pose );

    int index = 1;
    /*for ( int i = 0; i < inliers.rows; i++ ){
        int index = inliers.at<int> ( i,0 );
        g2o::VertexSBAPointXYZ* point = new g2o::VertexSBAPointXYZ();
        point->setId ( i );
        point->setEstimate ( Eigen::Vector3d (points_3d[index].x, points_3d[index].y, points_3d[index].z) );
        point->setMarginalized ( true );
        optimizer.addVertex ( point );
    }*/
    for ( const Point3f p:points_3d )   // landmarks
    {
        g2o::VertexSBAPointXYZ* point = new g2o::VertexSBAPointXYZ();
        point->setId ( index++ );
        point->setEstimate ( Eigen::Vector3d ( p.x, p.y, p.z ) );
        point->setMarginalized ( true );
        optimizer.addVertex ( point );
    }

    // parameter: camera intrinsics
    g2o::CameraParameters* camera = new g2o::CameraParameters (
        K.at<double> ( 0,0 ), Eigen::Vector2d ( K.at<double> ( 0,2 ), K.at<double> ( 1,2 ) ), 0
    );
    camera->setId ( 0 );
    optimizer.addParameter ( camera );

    // edges
    index = 1;
    for ( const Point2f p:points_2d )
    {
        g2o::EdgeProjectXYZ2UV* edge = new g2o::EdgeProjectXYZ2UV();
        edge->setId ( index );
        edge->setVertex ( 0, dynamic_cast<g2o::VertexSBAPointXYZ*> ( optimizer.vertex ( index ) ) );
        edge->setVertex ( 1, pose );
        edge->setMeasurement ( Eigen::Vector2d ( p.x, p.y ) );
        edge->setParameterId ( 0,0 );
        edge->setInformation ( Eigen::Matrix2d::Identity() );
        optimizer.addEdge ( edge );
        index++;
    }

    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    optimizer.setVerbose ( true );
    optimizer.initializeOptimization();
    optimizer.optimize (5);
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>> ( t2-t1 );
    cout<<"optimization costs time: "<<time_used.count() <<" seconds."<<endl;

    cout<<endl<<"after optimization:"<<endl;
    cout<<"T="<<endl<<Eigen::Isometry3d ( pose->estimate() ).matrix() <<endl;

}
void poseEstimationPnP(vector<KeyPoint>& keypoint_ref,vector<KeyPoint>& keypoint_cur,vector<float>& pose,cv::Mat& depth,vector<DMatch>& matches)
{
    Eigen::Quaterniond q(pose[1],pose[2],pose[3],pose[4]);
    Eigen::Matrix3d R;
    R = q.toRotationMatrix();
    Eigen::Vector3d t;
    t(0,0) = pose[5];
    t(1,0) = pose[6];
    t(2,0) = pose[7];

    Eigen::Vector3d local_point;
    Eigen::Vector3d global_point;

    //construct the 3d 2d observations
    vector<cv::Point3f> pts3d;
    vector<cv::Point2f> pts2d;

    for(int i = 0;i < matches.size();i++){
        pts2d.push_back(keypoint_cur[matches[i].queryIdx].pt);
       
        int image_u = (int)keypoint_ref[matches[i].trainIdx].pt.x;
        int image_v = (int)keypoint_ref[matches[i].trainIdx].pt.y;

        uint16_t Depth = reinterpret_cast<uint16_t>(depth.at<uint16_t>(image_v,image_u))/1000;
        cv::Point3f temp;
        temp.x = (image_u-cx)*Depth/fx; 
        temp.y = (image_v-cy)*Depth/fy;
        temp.z = Depth;
        
        local_point(0,0) = temp.x;
        local_point(1,0) = temp.y;
        local_point(2,0) = temp.z;

        global_point = R*local_point + t;

        pts3d.push_back(cv::Point3f(global_point(0,0),global_point(1,0),global_point(2,0)));
    }

    Mat K = ( cv::Mat_<double> ( 3,3 ) <<
              fx, 0, cx,
              0, fy, cy,
              0, 0, 1
            );

    Mat rvec, tvec, inliers;
    cv::solvePnPRansac ( pts3d, pts2d, K, Mat(), rvec, tvec, false, 100, 4.0, 0.99, inliers );
    int num_inliers_ = inliers.rows;
    cout<<"pnp inliers: "<<num_inliers_<<endl;
   
	Mat global_R;
    cv::Rodrigues (rvec,global_R ); 
    bundleAdjustment ( pts3d, pts2d, K, global_R, tvec,inliers);
}
bool GetRobotCurrentPose(
                         relocalization_sweeper::GetRobotPose::Request &req,
                         relocalization_sweeper::GetRobotPose::Response &res){
    ROS_INFO("start relocalization algorithm...");
    ROS_INFO("Load vocabulary...");
    
    ROS_INFO("Vocabulary Direction:%s",voc_dir.c_str());
    ROS_INFO("Data Direction:%s",dataset_dir.c_str());

    DBoW3::Vocabulary voc(voc_dir+"voc.yml.gz");
    //std::cout << __FILE__ << __LINE__ << std::endl;

    if(voc.empty()){
        ROS_ERROR("Load vocabulary fail...");
        return false;
    }
    
    std::cout << "Voc Info:" << voc << std::endl;
    //std::cout << __FILE__ << __LINE__ << std::endl;
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr =  cv_bridge::toCvCopy(req.img_data, sensor_msgs::image_encodings::MONO8);

    //std::cout << __FILE__ << __LINE__ << std::endl;
    cv::Mat source = cv_ptr->image;
    cv::Mat source_descriptor;
    vector<KeyPoint> source_keypoint; 

    ROS_INFO("Image Height:%d,Image Width:%d",source.rows,source.cols);
    
    ifstream fin ( dataset_dir+"/data.txt" );
    if ( !fin )
    {
        ROS_INFO("Read Data Fail...");
        return 1;
    }
    
    //read image
    vector<string> rgb_files;
    string rgb_file;
    while (getline(fin,rgb_file) && ros::ok())
    {
        rgb_files.push_back ( dataset_dir+rgb_file );
    }
    fin.close();

    for(int i = 0;i < rgb_files.size();i++)
        ROS_INFO("%s",rgb_files[i].c_str());

    vector<Mat> descriptors;
    vector<vector<KeyPoint> > keypoints;
    //Ptr< Feature2D > detector = ORB::create();
    int num_of_features = 256;
    double scale_factor = 1.2;
    int level_pyramid = 5;
    cv::Ptr<cv::ORB> detector = cv::ORB::create(num_of_features,scale_factor,level_pyramid);

    //calculate source keypoints and descriptor
    detector->detectAndCompute(source, Mat(), source_keypoint, source_descriptor);
    
    //int index = 1;
    //extract orb feature
    for ( string rgb_file:rgb_files )
    {
        Mat image = imread(rgb_file,0);

        if(image.empty())
            continue;

        vector<KeyPoint> keypoint; 
        Mat descriptor;
        detector->detectAndCompute( image, Mat(), keypoint, descriptor );
        descriptors.push_back( descriptor );
        keypoints.push_back(keypoint);
        //ROS_INFO("Extracting Features From Image:%d ",index++);
    }

    //construct database
    DBoW3::Database db( voc, false, 0);
    for ( int i=0; i<descriptors.size(); i++ )
        db.add(descriptors[i]);

    std::cout << "database info: " << db << std::endl;
    DBoW3::QueryResults ret;
    db.query(source_descriptor, ret, 3);

    ROS_INFO("Candidate Image:");
    //std::cout << "searching for source image" << ret << std::endl;
    ROS_INFO_STREAM_ONCE(ret);
    ROS_INFO("ret_id:%d",(int)ret[0].Id);
    ROS_INFO("ret_id:%d",(int)ret[1].Id);
    ROS_INFO("ret_id:%d",(int)ret[2].Id);
    //std::cout << __FILE__ << __LINE__ << std::endl;

    
    //calculate robot pose according to EPNP
    stringstream ss_depth;
    ss_depth << ret[0].Id;
    string s_depth = depth_dir+ss_depth.str()+"_depth.png"; 
    cv::Mat depth_frame = cv::imread(s_depth);

    if(depth_frame.empty())
        ROS_ERROR("Load Depth image fail...");

    std::ifstream infile(pose_dir+"pose_list.txt");
    std::vector<std::vector<float> > pose_vec;
    std::vector<float> temp_vec;
    if(infile.is_open()){
        std::string lenStr;
        while(getline(infile,lenStr)){
            unsigned int found = 0;
            int pos = 0;
            std::vector<float> tempVec;
            for(unsigned int i = 0;i < lenStr.length()/2;i++){
                found = lenStr.find(",",pos);
                std::string tempData = lenStr.substr(pos,found-pos);
                temp_vec.push_back(atof(tempData.c_str()));
                pos = found +1;
            }
            pose_vec.push_back(temp_vec);
        }
    }
    
    vector<float> robot_world_pose;
    robot_world_pose.swap(pose_vec[ret[0].Id]);

    std::vector<DMatch> matches;
    Mat descriptorMat_ref = descriptors[ret[0].Id];
    Mat descriptorMat_cur = source_descriptor;

    match_features_knn(descriptorMat_ref, descriptorMat_cur,matches);

    Mat homography;
    vector<KeyPoint> keyPoint_ref(keypoints[ret[0].Id]);
    vector<KeyPoint> keyPoint_cur(source_keypoint);
    refine_match_with_homography(keyPoint_ref,keyPoint_cur,3,matches,homography);

    poseEstimationPnP(keyPoint_ref,keyPoint_cur,robot_world_pose,depth_frame,matches);

    return true;
}

}


int main(int argc, char **argv) {
  ros::init(argc, argv, "relocalization_robot_service");

  ros::NodeHandle private_nh("~");
  private_nh.param<std::string>("voc_dir",voc_dir,"../voc/");
  private_nh.param<std::string>("dataset_dir",dataset_dir,"../data/");
  private_nh.param<double>("fx",fx,0);
  private_nh.param<double>("fy",fy,0);
  private_nh.param<double>("cx",cx,0);
  private_nh.param<double>("cy",cy,0);
  private_nh.param<std::string>("depth_dir",depth_dir,"../depth/");
  private_nh.param<std::string>("pose_dir",pose_dir,"../pose/");

  //advertise a service for getting a coverage plan
  ros::ServiceServer make_coverage_plan_srv = private_nh.advertiseService(
      "/sweeper/relocalization_robot_srv",
      relocalization_robot::GetRobotCurrentPose);

  ROS_INFO("Get Robot Current Pose Service Active...");

  ros::spin();

  return (0);
}


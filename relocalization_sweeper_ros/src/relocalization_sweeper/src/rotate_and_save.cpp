#include <ros/ros.h>
#include <tf/tf.h>
#include <std_msgs/Float32.h>

#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/calib3d/calib3d.hpp"

#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm> 

using namespace cv;
using namespace std;


typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,sensor_msgs::Image,sensor_msgs::CameraInfo, nav_msgs::Odometry> sync_pol;

class SaveCurrentImageAndRobotPose
{
public:  
	message_filters::Subscriber<sensor_msgs::Image>* rgb_sub;
	message_filters::Subscriber<sensor_msgs::Image>* depth_sub;
	message_filters::Subscriber<sensor_msgs::CameraInfo>* caminfo_sub;
    message_filters::Subscriber<nav_msgs::Odometry>* pose_sub;
	message_filters::Synchronizer<sync_pol>* sync;


  SaveCurrentImageAndRobotPose(ros::NodeHandle h,
                               string rt,
                               string dt,
                               string ct,
                               string pt,
                               string rgb_dir,
                               string depth_dir,
                               string pose_dir,
                               double robot_move_t,
                               double robot_rotate_t,
                               int feature_match_t)
                               :nh_(h),rgb_topic(rt),depth_topic(dt),camera_info_topic(ct),robot_pose_topic(pt),
                               rgb_directory(rgb_dir),depth_directory(depth_dir),pose_directory(pose_dir),
                               robot_move_th(robot_move_t),robot_rotate_th(robot_rotate_t),feature_match_th(feature_match_t)
  {
        frame_index = 0;

		rgb_sub = new message_filters::Subscriber<sensor_msgs::Image>(nh_, rgb_topic, 1);
		depth_sub = new message_filters::Subscriber<sensor_msgs::Image>(nh_, depth_topic, 1);
		caminfo_sub = new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh_, camera_info_topic, 1);
		pose_sub = new message_filters::Subscriber<nav_msgs::Odometry>(nh_, robot_pose_topic, 1);
		sync = new  message_filters::Synchronizer<sync_pol>(sync_pol(10), *rgb_sub,*depth_sub,*caminfo_sub,*pose_sub);

		sync->registerCallback(boost::bind(&SaveCurrentImageAndRobotPose::analysisCB,this,_1,_2,_3,_4));

  }

  ~SaveCurrentImageAndRobotPose(){
    delete[] rgb_sub;
    delete[] caminfo_sub;
    delete[] pose_sub;
    delete[] depth_sub;
  }

  void analysisCB(const sensor_msgs::ImageConstPtr& msg_rgb,
                  const sensor_msgs::ImageConstPtr& msg_depth,
                  const sensor_msgs::CameraInfoConstPtr& msg_cam_info,
                  const nav_msgs::OdometryConstPtr& msg_robot_pose);
  bool addKeyFrame();
  bool countRefAndCurFrame();
  bool crossRefAndCurPose();
  void match_features_knn(Mat& query,Mat& train,vector<DMatch>& matches);
  void refine_match_with_homography(vector<KeyPoint>& queryKeyPoints,
                                    vector<KeyPoint>& trainKeyPoints,
                                    float reprojectionThreshold,
                                    vector<DMatch>& matches,
                                    Mat& homography);

protected:
   
  ros::NodeHandle nh_;
  std::string rgb_topic;
  std::string depth_topic;
  std::string camera_info_topic;
  std::string robot_pose_topic;

  int frame_index;
  cv::Mat ref_frame;
  geometry_msgs::PoseStamped ref_pose;
  cv::Mat cur_frame;
  geometry_msgs::PoseStamped cur_pose;

  string rgb_directory;
  string depth_directory;
  string pose_directory;

  double robot_move_th;
  double robot_rotate_th;
  int feature_match_th;

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "save_image_and_robot_pose");

  ros::NodeHandle n,pn("~");

  string rgb_t;
  string depth_t;
  string camera_info_t;
  string robot_pose_t;
  string rgb_dir;
  string depth_dir;
  string robot_pose_dir;
  double robot_move_threshold;
  double robot_rotate_threshold;
  int feature_match_threshold;

  pn.param<string>("rgb_topic",rgb_t,"");
  pn.param<string>("depth_topic",depth_t,"");
  pn.param<string>("camera_info_topic",camera_info_t,"");
  pn.param<string>("robot_pose_topic",robot_pose_t,"");
  pn.param<string>("rgb_dir",rgb_dir,"../rgb/");
  pn.param<string>("depth_dir",depth_dir,"../depth/");
  pn.param<string>("robot_pose_dir",robot_pose_dir,"../pose/");
  pn.param<double>("robot_move_threshold",robot_move_threshold,0.3);
  pn.param<double>("robot_rotate_threshold",robot_rotate_threshold,10.0);
  pn.param<int>("feature_match_threshold",feature_match_threshold,30);

  ROS_INFO("robot_pose_dir:%s",robot_pose_dir.c_str());
  ROS_INFO("depth_dir:%s",depth_dir.c_str());
  ROS_INFO("rgb_dir:%s",rgb_dir.c_str());
  ROS_INFO("robot_pose_topic:%s",robot_pose_t.c_str());
  ROS_INFO("rgb_topic:%s",rgb_t.c_str());
  ROS_INFO("depth_topic:%s",depth_t.c_str());
  ROS_INFO("robot_move_threshold:%f",robot_move_threshold);
  ROS_INFO("robot_rotate_threshold:%f",robot_rotate_threshold);
  ROS_INFO("feature_match_threshold:%d",feature_match_threshold);

  SaveCurrentImageAndRobotPose obj(n,
                                   rgb_t,
                                   depth_t,
                                   camera_info_t,
                                   robot_pose_t,
                                   rgb_dir,
                                   depth_dir,
                                   robot_pose_dir,
                                   robot_move_threshold,
                                   robot_rotate_threshold,
                                   feature_match_threshold);
  ros::spin();

  return 0;
}
void SaveCurrentImageAndRobotPose::match_features_knn(Mat& query, Mat& train, vector<DMatch>& matches)
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
void SaveCurrentImageAndRobotPose::refine_match_with_homography(vector<KeyPoint>& queryKeyPoints,
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
bool SaveCurrentImageAndRobotPose::countRefAndCurFrame()
{   
    CV_Assert(ref_frame.data != NULL && cur_frame.data != NULL);
    std::vector<KeyPoint> keyPoint_ref, keyPoint_cur;

    int num_of_features = 256;
    double scale_factor = 1.2;
    int level_pyramid = 5;

    cv::Ptr<cv::ORB> orb; 
    orb = cv::ORB::create(num_of_features,scale_factor,level_pyramid);

    orb->detect(ref_frame, keyPoint_ref);
    orb->detect(cur_frame, keyPoint_cur);

    Mat descriptorMat_ref, descriptorMat_cur;
    orb->compute(ref_frame, keyPoint_ref, descriptorMat_ref);
    orb->compute(cur_frame, keyPoint_cur, descriptorMat_cur);

    std::vector<DMatch> matches;
    match_features_knn(descriptorMat_ref, descriptorMat_cur,matches);

    if(matches.size() < feature_match_th){
        ROS_INFO("feature match enough...");
        ROS_INFO("matches_size:%d",(int)matches.size());
        return true;
    }
    else{
        Mat homography;
        refine_match_with_homography(keyPoint_ref,keyPoint_cur,3,matches,homography);
    
        if(matches.size() < feature_match_th){
            ROS_INFO("feature match homography enough...");
            ROS_INFO("matches_size_homography:%d",(int)matches.size());
            return true;
        }
        else
            return false;
    }
}
bool SaveCurrentImageAndRobotPose::crossRefAndCurPose()
{
    tf::Transform ref_t;
    ref_t.setOrigin(tf::Vector3(ref_pose.pose.position.x, ref_pose.pose.position.y, ref_pose.pose.position.z));
    ref_t.setRotation(tf::Quaternion(ref_pose.pose.orientation.x,ref_pose.pose.orientation.y,ref_pose.pose.orientation.z,ref_pose.pose.orientation.w));

    tf::Transform cur_t;
    cur_t.setOrigin(tf::Vector3(cur_pose.pose.position.x, cur_pose.pose.position.y, cur_pose.pose.position.z));
    cur_t.setRotation(tf::Quaternion(cur_pose.pose.orientation.x,cur_pose.pose.orientation.y,cur_pose.pose.orientation.z,cur_pose.pose.orientation.w));

    tf::Transform delta = ref_t.inverse()*cur_t;

    if(sqrt(delta.getOrigin().getX()*delta.getOrigin().getX()+delta.getOrigin().getY()*delta.getOrigin().getY()) > robot_move_th \
       || fabs(tf::getYaw(delta.getRotation())*180/M_PI) > robot_rotate_th){
        ROS_INFO("robot_move_distance:%f",\
            sqrt(delta.getOrigin().getX()*delta.getOrigin().getX()+delta.getOrigin().getY()*delta.getOrigin().getY()));
        ROS_INFO("robot_rotate_angle:%f",fabs(tf::getYaw(delta.getRotation())*180/M_PI));
        ROS_INFO("robot move or rotate enough...");
        return true;
    }
    else
        return false;
}   
bool SaveCurrentImageAndRobotPose::addKeyFrame()
{
    if(countRefAndCurFrame())
        return true;
    else{
        if(crossRefAndCurPose())
            return true;
        else
            return false;
    }
}
void SaveCurrentImageAndRobotPose::analysisCB(const sensor_msgs::ImageConstPtr& msg_rgb,
                                              const sensor_msgs::ImageConstPtr& msg_depth,
                                              const sensor_msgs::CameraInfoConstPtr& msg_cam_info,
                                              const nav_msgs::OdometryConstPtr& msg_robot_pose)
  {
    
    cv_bridge::CvImagePtr cv_ptr_rgb;
    cv_ptr_rgb =  cv_bridge::toCvCopy(msg_rgb, sensor_msgs::image_encodings::MONO8);
    //std::cout << __FILE__ << __LINE__ << std::endl;
    cv::Mat image_rgb = cv_ptr_rgb->image;
    cur_frame = image_rgb;

    cur_pose.header = msg_robot_pose->header;
    cur_pose.pose = msg_robot_pose->pose.pose;
    
    if(cur_frame.empty()){
        ROS_ERROR("Get Current frame fail...");
    }

    ROS_INFO("start save image and robot pose...");  
    if(frame_index == 0){
        stringstream ss_rgb;
        ss_rgb << frame_index;
        string s_rgb = rgb_directory+ss_rgb.str()+"_rgb.png";

        ofstream outfile_data;
        outfile_data.open(rgb_directory+"data.txt",ios::out);
        outfile_data << ss_rgb.str()+"_rgb.png" << std::endl;
        outfile_data.close();

        stringstream ss_depth;
        ss_depth << frame_index;
        string s_depth = depth_directory+ss_depth.str()+"_depth.png"; 
        
        /*cv_bridge::CvImagePtr cv_ptr_rgb;
        cv_ptr_rgb =  cv_bridge::toCvCopy(msg_rgb, sensor_msgs::image_encodings::MONO8);
        //std::cout << __FILE__ << __LINE__ << std::endl;
        cv::Mat image_rgb = cv_ptr_rgb->image;*/
        cv::imwrite(s_rgb,cur_frame);
        
        //ref_frame = cur_frame;
        //frame_index++;

        cv_bridge::CvImagePtr cv_ptr_depth;
        cv_ptr_depth =  cv_bridge::toCvCopy(msg_depth, sensor_msgs::image_encodings::TYPE_16UC1);
        //std::cout << __FILE__ << __LINE__ << std::endl;
        cv::Mat image_depth = cv_ptr_depth->image;
        imwrite(s_depth,image_depth);

        //ref_pose.header = msg_robot_pose->header;
        //ref_pose.pose = msg_robot_pose->pose;
        ref_pose = cur_pose;
        ref_frame = cur_frame;
        
        double fx,fy,cx,cy;
        fx = msg_cam_info->K[0];
        fy = msg_cam_info->K[4];

        cx = msg_cam_info->K[2];
        cy = msg_cam_info->K[5];

        double tx,ty,tz;
        double rx,ry,rz,w;
        
        tx = cur_pose.pose.position.x;
        ty = cur_pose.pose.position.y;
        tz = cur_pose.pose.position.z;

        rx = cur_pose.pose.orientation.x;
        ry = cur_pose.pose.orientation.y;
        rz = cur_pose.pose.orientation.z;
        w = cur_pose.pose.orientation.w;

        ofstream outfile;
        outfile.open(pose_directory,ios::out);
        if(!outfile.is_open())
            ROS_ERROR("Open file_empty failure...");
        else{
            outfile << frame_index << "," \
                << w << "," \
                << rx << ","\
                << ry << ","\
                << rz << ","\
                << tx << ","\
                << ty << ","\
                << tz << ","\
                << fx << ","\
                << fy << ","\
                << cx << ","\
                << cy << std::endl;

            frame_index++;
            outfile.close();
        }
        /*ofstream outfile;
        outfile.open(pose_directory,ios::app);
        if(!outfile.is_open())
           ROS_ERROR("Open file failure...");
        else{
            outfile << frame_index << "," \
                << w << "," \
                << rx << ","\
                << ry << ","\
                << rz << ","\
                << tx << ","\
                << ty << ","\
                << tz << std::endl;

            outfile.close();
            frame_index++;
        }*/

    }
    else{
        if(addKeyFrame()){
            stringstream ss_rgb;
            ss_rgb << frame_index;
            string s_rgb = rgb_directory+ss_rgb.str()+"_rgb.png";

            ofstream outfile_data;
            outfile_data.open(rgb_directory+"data.txt",ios::app);
            outfile_data << ss_rgb.str()+"_rgb.png" << std::endl;
            outfile_data.close();

            stringstream ss_depth;
            ss_depth << frame_index;
            string s_depth = depth_directory+ss_depth.str()+"_depth.png"; 
        
            /*cv_bridge::CvImagePtr cv_ptr_rgb;
            cv_ptr_rgb =  cv_bridge::toCvCopy(msg_rgb, sensor_msgs::image_encodings::MONO8);
            //std::cout << __FILE__ << __LINE__ << std::endl;
            cv::Mat image_rgb = cv_ptr_rgb->image;*/
            cv::imwrite(s_rgb,cur_frame);
        
            //ref_frame = cur_frame;
            //frame_index++;
            //ref_pose = cur_pose;

            cv_bridge::CvImagePtr cv_ptr_depth;
            cv_ptr_depth =  cv_bridge::toCvCopy(msg_depth, sensor_msgs::image_encodings::TYPE_16UC1);
            //std::cout << __FILE__ << __LINE__ << std::endl;
            cv::Mat image_depth = cv_ptr_depth->image;
            imwrite(s_depth,image_depth);

            ref_frame = cur_frame;
            ref_pose = cur_pose;

            double fx,fy,cx,cy;
            fx = msg_cam_info->K[0];
            fy = msg_cam_info->K[4];

            cx = msg_cam_info->K[2];
            cy = msg_cam_info->K[5];

            double tx,ty,tz;
            double rx,ry,rz,w;

            tx = cur_pose.pose.position.x;
            ty = cur_pose.pose.position.y;
            tz = cur_pose.pose.position.z;

            rx = cur_pose.pose.orientation.x;
            ry = cur_pose.pose.orientation.y;
            rz = cur_pose.pose.orientation.z;
            w = cur_pose.pose.orientation.w;
            
            ofstream outfile;
            outfile.open(pose_directory,ios::app);
            if(!outfile.is_open())
                ROS_ERROR("Open file failure...");
            else{
                outfile << frame_index << "," \
                    << w << "," \
                    << rx << ","\
                    << ry << ","\
                    << rz << ","\
                    << tx << ","\
                    << ty << ","\
                    << tz << ","\
                    << fx << ","\
                    << fy << ","\
                    << cx << ","\
                    << cy << std::endl;
                    outfile.close();
                    frame_index++;
            }
        }
    }

  }


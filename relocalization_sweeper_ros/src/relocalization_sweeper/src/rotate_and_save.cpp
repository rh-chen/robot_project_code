#include <ros/ros.h>
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

#include <string>
#include <iostream>
#include <sstream>
#include <algorithm> 

using namespace cv;
using namespace std;


typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,sensor_msgs::Image,sensor_msgs::CameraInfo,geometry_msgs::PoseStamped> sync_pol;

class SaveCurrentImageAndRobotPose
{
public:  
	message_filters::Subscriber<sensor_msgs::Image>* rgb_sub;
	message_filters::Subscriber<sensor_msgs::Image>* depth_sub;
	message_filters::Subscriber<sensor_msgs::CameraInfo>* caminfo_sub;
    message_filters::Subscriber<geometry_msgs::PoseStamped>* pose_sub;
	message_filters::Synchronizer<sync_pol>* sync;


  SaveCurrentImageAndRobotPose(ros::NodeHandle h,
                               string rt,
                               string dt,
                               string ct,
                               string pt,
                               string rgb_dir,
                               string depth_dir,
                               string pose_dir)
                               :nh_(h),rgb_topic(rt),depth_topic(dt),camera_info_topic(ct),robot_pose_topic(pt),
                               rgb_directory(rgb_dir),depth_directory(depth_dir),pose_directory(pose_dir)
  {
        image_index = 0;

		rgb_sub = new message_filters::Subscriber<sensor_msgs::Image>(nh_, rgb_topic, 1);
		depth_sub = new message_filters::Subscriber<sensor_msgs::Image>(nh_, depth_topic, 1);
		caminfo_sub = new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh_, camera_info_topic, 1);
		pose_sub = new message_filters::Subscriber<geometry_msgs::PoseStamped>(nh_, robot_pose_topic, 1);
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
                  const geometry_msgs::PoseStampedConstPtr& msg_robot_pose);
  bool addKeyFrame();

protected:
   
  ros::NodeHandle nh_;
  std::string rgb_topic;
  std::string depth_topic;
  std::string camera_info_topic;
  std::string robot_pose_topic;

  int image_index;
  cv::Mat ref_frame;
  geometry_msgs::PoseStamped ref_pose;
  cv::Mat cur_frame;
  geometry_msgs::PoseStamped cur_pose;

  string rgb_directory;
  string depth_directory;
  string pose_directory;

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

  pn.param<string>("rgb_topic",rgb_t,"");
  pn.param<string>("depth_topic",depth_t,"");
  pn.param<string>("camera_info_topic",camera_info_t,"");
  pn.param<string>("robot_pose_topic",robot_pose_t,"");
  pn.param<string>("rgb_dir",rgb_dir,"");
  pn.param<string>("depth_dir",depth_dir,"");
  pn.param<string>("robot_pose_dir",robot_pose_dir,"");

  SaveCurrentImageAndRobotPose obj(n,
                                   rgb_t,
                                   depth_t,
                                   camera_info_t,
                                   robot_pose_t,
                                   rgb_dir,
                                   depth_dir,
                                   pose_dir);
  ros::spin();

  return 0;
}

bool SaveCurrentImageAndRobotPose::countRefAndCurFrame()
{
   
    
   if(match.size() < threshold)
        return true;
   else
        return false;
}
bool SaveCurrentImageAndRobotPose::crossRefAndCurPose()
{
    geometry_msgs::Pose ref = ref_pose.pose;
    geometry_msgs::Pose cur = cur_pose.pose;

    geometry_msgs::Pose delta = ref.inverse()*cur;

    if(sqrt(delta.position.x*delta.position.x+delta.position.y*delta.position.y) > robot_move_threshold \
       || fabs(tf::getYaw(delta.orientation)*180/M_PI) > robot_rotate_threshold)
        return true;
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
                                              const geometry_msgs::PoseStampedConstPtr& msg_robot_pose)
  {
    
    cv_bridge::CvImagePtr cv_ptr_rgb;
    cv_ptr_rgb =  cv_bridge::toCvCopy(msg_rgb, sensor_msgs::image_encodings::MONO8);
    //std::cout << __FILE__ << __LINE__ << std::endl;
    cv::Mat image_rgb = cv_ptr_rgb->image;

    cur_frame = image_rgb;

    ROS_INFO("start save image and robot pose...");  
    if(image_index == 0){
        stringstream ss_rgb;
        ss_rgb << image_index;
        string s_rgb = rgb_directory+ss_rgb.str()+"_rgb.png";

        stringstream ss_depth;
        ss_depth << image_index;
        string s_depth = depth_directory+ss_depth.str()+"_depth.png"; 
        
        /*cv_bridge::CvImagePtr cv_ptr_rgb;
        cv_ptr_rgb =  cv_bridge::toCvCopy(msg_rgb, sensor_msgs::image_encodings::MONO8);
        //std::cout << __FILE__ << __LINE__ << std::endl;
        cv::Mat image_rgb = cv_ptr_rgb->image;*/
        imwrite(s_rgb,cur_frame);
        
        ref_frame = cur_frame;

        cv_bridge::CvImagePtr cv_ptr_depth;
        cv_ptr_depth =  cv_bridge::toCvCopy(msg_depth, sensor_msgs::image_encodings::TYPE_16UC1);
        //std::cout << __FILE__ << __LINE__ << std::endl;
        cv::Mat image_depth = cv_ptr_depth->image;
        imwrite(s_depth,image_depth);

        double tx,ty,tz;
        double rx,ry,rz,w;
        
        ofstream outfile;
        outfile.open(depth_directory,ios::app);
        if(!outfile.is_open())
           ROS_ERROR("Open file failure...");

        outfile << image_index << "\t" << tx << "\t" << ty << "\t" << tz << "\t" << rx << "\t" << ry << rz << "\t" << w << std::endl;

    }
    else{
        if(checkKeyFrame()){
            ref_frame = cur_frame;

            ofstream outfile;
            outfile.open(depth_directory,ios::app);
            if(!outfile.is_open())
                ROS_ERROR("Open file failure...");
            else{
                outfile << image_index << "\t" << tx << "\t" << ty << "\t" << tz << "\t" << rx << "\t" << ry << rz << "\t" << w << std::endl;
                frame_index++;
                outfile.close();
            }
        }
    }

  }


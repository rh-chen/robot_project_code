#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <actionlib/server/simple_action_server.h>
#include <ret_chargeable_pile/FindMarkerPoseAction.h>


#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <nav_msgs/Odometry.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/core/core.hpp>

#include <string>
#include <iostream>
#include <algorithm> 

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::CameraInfo> sync_pol;

class FindMarkerPoseAction
{
public:  
	message_filters::Subscriber<sensor_msgs::Image>* rgb_sub;
	message_filters::Subscriber<sensor_msgs::CameraInfo>* caminfo_sub;
	message_filters::Synchronizer<sync_pol>* sync;


  FindMarkerPoseAction(std::string name) : 
    as_(nh_, name, false),
    action_name_(name)
  {
		rgb_sub = new message_filters::Subscriber<sensor_msgs::Image>(nh_, "/left/rgb/image_raw_color", 1);
		caminfo_sub = new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh_, "/left/rgb/cam_info", 1);
		sync = new  message_filters::Synchronizer<sync_pol>(sync_pol(10), *rgb_sub,*caminfo_sub);

		sync->registerCallback(boost::bind(&FindMarkerPoseAction::analysisCB,this,_1,_2));

    //register the goal and feeback callbacks
    as_.registerPreemptCallback(boost::bind(&FindMarkerPoseAction::preemptCB, this));

    //subscribe to the data topic of interest
    //sub_ = nh_.subscribe("/random_number", 1, &FindMarkerPoseAction::analysisCB, this);//
    as_.start();
  }

  ~FindMarkerPoseAction(void)
  {
  }

	void analysisCB(const sensor_msgs::ImageConstPtr& msg_rgb,const sensor_msgs::CameraInfoConstPtr& msg_cam_info);

  void goalCB()
  {
    // reset helper variables
    //data_count_ = 0;//
    //sum_ = 0;//
    //sum_sq_ = 0;//
    // accept the new goal
    //goal_ = as_.acceptNewGoal()->samples;//
  }

  void preemptCB()
  {
    ROS_INFO("%s: Preempted", action_name_.c_str());
    // set the action state to preempted
    as_.setPreempted();
  }

protected:
   
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<ret_chargeable_pile::FindMarkerPoseAction> as_;
  std::string action_name_;

  ret_chargeable_pile::FindMarkerPoseFeedback feedback_;
  ret_chargeable_pile::FindMarkerPoseResult result_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "FindMarkerPose");

  FindMarkerPoseAction fmp(ros::this_node::getName());
  ros::spin();

  return 0;
}

void FindMarkerPoseAction::analysisCB(const sensor_msgs::ImageConstPtr& msg_rgb,const sensor_msgs::CameraInfoConstPtr& msg_cam_info)
  {
    // make sure that the action hasn't been canceled
    if (!as_.isActive())
      return;
    
    /*data_count_++;
    feedback_.sample = data_count_;
    feedback_.data = msg->data;
    //compute the std_dev and mean of the data 
    sum_ += msg->data;
    feedback_.mean = sum_ / data_count_;
    sum_sq_ += pow(msg->data, 2);
    feedback_.std_dev = sqrt(fabs((sum_sq_/data_count_) - pow(feedback_.mean, 2)));
    as_.publishFeedback(feedback_);

    if(data_count_ > goal_) 
    {
      result_.mean = feedback_.mean;
      result_.std_dev = feedback_.std_dev;

      if(result_.mean < 5.0)
      {
        ROS_INFO("%s: Aborted", action_name_.c_str());
        //set the action state to aborted
        as_.setAborted(result_);
      }
      else 
      {
        ROS_INFO("%s: Succeeded", action_name_.c_str());
        // set the action state to succeeded
        as_.setSucceeded(result_);
      }
    } */
		ROS_INFO("configure	action server is ok!");
  }


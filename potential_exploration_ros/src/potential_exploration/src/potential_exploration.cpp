#include <deque>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
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

#include <boost/thread/thread.hpp>
#include <boost/make_shared.hpp>

#include <stdio.h>
#include <math.h>

#include <hash_set>
#include <set>
#include <map>
#include <algorithm> 

#include <tf/tf.h>
#include "potential_exploration/GetNextFrontier.h"


namespace potential_exploration_ns{

	bool GetNextFrontier(potential_exploration::GetNextFrontier::Request &req,
						 potential_exploration::GetNextFrontier::Response &res){

		std::cout << "start potential exploration..." << std::endl;
        return true;
	}
}
int main(int argc, char **argv) {
  ros::init(argc, argv, "potential_exploration_node");

  ros::NodeHandle private_nh("~");

  ros::ServiceServer potential_exploration_srv = private_nh.advertiseService(
      "/sweeper/potential_exploration",
      potential_exploration_ns::GetNextFrontier);

  ROS_INFO("Ready to get next frontier...");

  ros::spin();

  return (0);
}

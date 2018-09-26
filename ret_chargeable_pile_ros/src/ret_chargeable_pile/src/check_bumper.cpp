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
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <kobuki_msgs/BumperEvent.h>
#include <boost/thread/thread.hpp>
#include <boost/make_shared.hpp>

#include <stdio.h>
#include <math.h>

#include <hash_set>
#include <set>
#include <map>
#include <algorithm> 

#include <tf/tf.h>
#include "ret_chargeable_pile/CheckBumper.h"

#define L_PI 3.1415926

using namespace std;
class CheckBumper{
	public:
        bool bumper_is_active;

        boost::mutex mtx;

        ros::NodeHandle n;
        ros::Subscriber bumper_sub;
        ros::ServiceServer bumper_srv;
        
        CheckBumper();
        void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg);
        bool checkBumperCallback(ret_chargeable_pile::CheckBumper::Request& req,
                                 ret_chargeable_pile::CheckBumper::Response& res);

};

CheckBumper::CheckBumper(){
    bumper_is_active = false;
    bumper_sub = n.subscribe("/mobile_base/events/bumper", 1000,&CheckBumper::bumperCallback,this);
    bumper_srv = n.advertiseService("/sweeper/CheckBumper",&CheckBumper::checkBumperCallback,this);
}
void CheckBumper::bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg){
    mtx.lock();
    if(msg->state == msg->PRESSED)
        bumper_is_active = true;
    else
        bumper_is_active = false;
    mtx.unlock();
}


bool CheckBumper::checkBumperCallback(ret_chargeable_pile::CheckBumper::Request& req,
                                      ret_chargeable_pile::CheckBumper::Response& res){
    if(req.check)
        res.active = bumper_is_active;

    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "check_bumper_node");
    CheckBumper cb;

    ros::spin();
    return 0;
}


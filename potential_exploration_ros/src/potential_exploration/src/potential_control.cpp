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
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include "visualization_msgs/Marker.h"

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
#include <potential_exploration/PotentialPlanner.h>
#include <potential_exploration/CollisionChecker.h>
#include <potential_exploration/WaypointProposition.h>
#define L_PI 3.1415926

using namespace std;

class robot_control{
    public:
        boost::mutex mtx;
        ros::NodeHandle node_handle;

        geometry_msgs::Pose2D current_waypoint;
        vector<geometry_msgs::Pose2D> current_path;

        ros::Publisher driver_pub;
        ros::ServiceServer waypoint_proposition_srv;

        robot_control();
        void waypointCallback();
        void planPathAndFollow();
        bool isPathStillSafe();
        void waypointCallback();
};

robot_control::robot_control(){
   driver_pub = node_handle.advertise<geometry_msgs::Pose2D[]>("/controller_path",1);
   waypoint_proposition_srv = \
                    node_handle.advertiseService("/sweeper/WaypointProposition", &robot_control::waypointCallback, this);
}

void robot_control::planPathAndFollow(){
    mtx.lock();
    
    ros::ServiceClient pp_client = \
                       node_handle.serviceClient<potential_exploration::PotentialPlanner>("/sweeper/PotentialPlanner");
    potential_exploration::PotentialPlanner pp_srv;
    pp_srv.request.goal_state_x = current_waypoint.x;
    pp_srv.request.goal_state_y = current_waypoint.y;
    
    bool pp_res = pp_client.call(pp_srv);
    if(pp_res)
        
    mtx.unlock();
}
void waypointCallback(){
    mtx.lock();

    mtx.unlock();
}
int main(int argc, char **argv) {
    ros::init(argc, argv, "potential_control_node");

    return 0;
}



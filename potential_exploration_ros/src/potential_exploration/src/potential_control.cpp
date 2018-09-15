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
#include "std_msgs/Bool.h"
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
#include <potential_exploration/Pose2DArray.h>
#define L_PI 3.1415926

using namespace std;

class robot_control{
    public:
        boost::mutex mtx;
        ros::NodeHandle node_handle;

        geometry_msgs::Pose2D current_waypoint;
        vector<geometry_msgs::Pose2D> current_path;

        ros::Publisher driver_pub;
        ros::Publisher replan_pub;

        ros::ServiceServer waypoint_proposition_srv;

        robot_control();
        bool planPathAndFollow();
        bool isPathStillSafe();
        bool waypointCallback(potential_exploration::WaypointProposition::Request& req,
                              potential_exploration::WaypointProposition::Response& res);
        void requestNewWaypoint();
};

robot_control::robot_control(){
   current_waypoint = geometry_msgs::Pose2D();

   driver_pub = node_handle.advertise<potential_exploration::Pose2DArray>("/controller_path",1);
   replan_pub = node_handle.advertise<std_msgs::Bool>("/mission_replan",1);

   waypoint_proposition_srv = \
                    node_handle.advertiseService("/sweeper/WaypointProposition", &robot_control::waypointCallback, this);
}

bool robot_control::planPathAndFollow(){
    mtx.lock();
    
    ros::ServiceClient pp_client = \
                       node_handle.serviceClient<potential_exploration::PotentialPlanner>("/sweeper/PotentialPlanner");
    potential_exploration::PotentialPlanner pp_srv;
    pp_srv.request.goal_state_x = current_waypoint.x;
    pp_srv.request.goal_state_y = current_waypoint.y;
    
    bool pp_res = pp_client.call(pp_srv);
    if(pp_res){
        for(int i = 0;i < pp_srv.response.poses.size();i++){
            current_path.push_back(pp_srv.response.poses[i]);
        }
        potential_exploration::Pose2DArray msg;
        msg.poses = pp_srv.response.poses;
        driver_pub.publish(msg);
        mtx.unlock();
        return true;
    }
    else{
        mtx.unlock();
        return false;
    }
}
bool robot_control::waypointCallback(potential_exploration::WaypointProposition::Request& req,
                                     potential_exploration::WaypointProposition::Response& res){
    mtx.lock();
    current_waypoint.x = req.waypoint.x;
    current_waypoint.y = req.waypoint.y;
    mtx.unlock();
    bool plan_path_res = planPathAndFollow();
    
    if(plan_path_res){
        res.accepted = true;
        return true;
    }
    else
        return false;
}
bool robot_control::isPathStillSafe(){
    mtx.lock();

    if(current_path.size() <= 0){
        mtx.unlock();
        return true;
    }

    ros::ServiceClient ck_client = \
                       node_handle.serviceClient<potential_exploration::CollisionChecker>("/sweeper/CollisionChecker");
    potential_exploration::CollisionChecker ck_srv;

    ck_srv.request.poses = current_path;
    
    bool ck_res = ck_client.call(ck_srv);

    if(ck_res)
        if(ck_srv.response.path_safe)
            return true;
        else
            return false;
    else
        return false;

}
void robot_control::requestNewWaypoint(){
   std_msgs::Bool bl;
   bl.data = true;

   replan_pub.publish(bl);
}
int main(int argc, char **argv) {
    ros::init(argc, argv, "potential_control_node");
    
    robot_control rc;

    ros::Rate r(100);
    while(ros::ok()){
        if(!rc.isPathStillSafe()){
            int num_tries = 0;
            while(!rc.planPathAndFollow()){
                ros::Duration(0.5).sleep();
                num_tries += 1;

                if(num_tries == 3){
                    ROS_INFO("The Waypoint unreachable,mission replan...");
                    rc.requestNewWaypoint();
                    break;
                }
            }
        }
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}



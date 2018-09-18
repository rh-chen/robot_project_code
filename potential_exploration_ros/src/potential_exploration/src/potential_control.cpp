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
	boost::mutex mtx1;
	boost::mutex mtx2;
	boost::mutex mtx3;

        ros::NodeHandle node_handle;

        geometry_msgs::Pose2D current_waypoint;
        vector<geometry_msgs::Pose2D> current_path;

        ros::Publisher driver_pub;
        ros::Publisher replan_pub;

        ros::ServiceServer waypoint_proposition_srv;
	ros::ServiceClient pp_client;
	ros::ServiceClient ck_client;

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
   pp_client = \
          node_handle.serviceClient<potential_exploration::PotentialPlanner>("/sweeper/PotentialPlanner");
   ck_client = \
          node_handle.serviceClient<potential_exploration::CollisionChecker>("/sweeper/CollisionChecker");
}

bool robot_control::planPathAndFollow(){
    mtx.lock();
   
    vector<geometry_msgs::Pose2D>().swap(current_path); 
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
    mtx1.lock();
    ROS_INFO("way point call back...");
    ROS_INFO("req.waypoint.x,req.waypoint.y:%f,%f",req.waypoint.x,req.waypoint.y);
    current_waypoint.x = req.waypoint.x;
    current_waypoint.y = req.waypoint.y;

    bool plan_path_res = false;
    plan_path_res = planPathAndFollow();
    
    if(plan_path_res){
        res.accepted = true;
	ROS_INFO("call /sweeper/PotentialPlanner success...");
	mtx1.unlock();
        return true;
    }
    else{
	res.accepted = false;
	ROS_INFO("call /sweeper/PotentialPlanner fail...");
	mtx1.unlock();
        return false;
    }
}
bool robot_control::isPathStillSafe(){
    mtx2.lock();
    ROS_INFO("call path still safe...");
    ROS_INFO("current_path_size:%d",(int)current_path.size());
    if(current_path.size() <= 0){
        mtx2.unlock();
        return true;
    }

    potential_exploration::CollisionChecker ck_srv;

    ck_srv.request.poses = current_path;
    
    bool ck_res = ck_client.call(ck_srv);

    if(ck_res)
        if(ck_srv.response.path_safe){
	    mtx2.unlock();
            return true;
	}
        else{
	    mtx2.unlock();
            return false;
	}
    else{
	mtx2.unlock();
        return false;
    }
}
void robot_control::requestNewWaypoint(){
   std_msgs::Bool bl;
   bl.data = true;

   replan_pub.publish(bl);
}
int main(int argc, char **argv) {
    ros::init(argc, argv, "potential_control_node");
    
    robot_control rc;

    ros::Rate r(4);
    while(ros::ok()){
	ROS_INFO("potential control loop ...");
        if(!rc.isPathStillSafe()){
	    ROS_INFO("path not safe,recalcalating...");
            int num_tries = 0;
            while(!rc.planPathAndFollow()){
                ros::Duration(0.4).sleep();
                num_tries += 1;

                if(num_tries == 3){
                    ROS_INFO("The Waypoint unreachable,mission replan...");
                    rc.requestNewWaypoint();
                    break;
                }
            }
        }
	/*else{
	    ROS_INFO("publishing current path...");
	    potential_exploration::Pose2DArray msg;
	    for(int i = 0;i < rc.current_path.size();i++){
                msg.poses.push_back(rc.current_path[i]);
            }
	    
            rc.driver_pub.publish(msg);
	}*/
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}



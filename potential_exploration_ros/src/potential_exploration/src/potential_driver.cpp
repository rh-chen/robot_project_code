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

class potential_driver{
   public:
        boost::mutex mtx;

        ros::NodeHandle node_handle;
        ros::Publisher vel_pub;
        ros::Publisher rep_pub;

        ros::Subscriber robot_pos_sub; 
        ros::Subscriber con_path_sub;

        double goal_th_xy;
        double inactive_thresh;
        double max_lin_speed;
        double max_ang_speed;

        int active_goal;
        bool follow_path;
        ros::Time last_inactive;

        double position_x;
        double position_y;
        double position_theta;

        vector<double> goals_x;
        vector<double> goals_y;
        vector<double> goals_theta;

        geometry_msgs::Twist vmsg;
        
        potential_driver();
        double angle_wrap(double angle);
        void odometryCallback(const nav_msgs::Odometry& msg);
        void next_goal();
        void clearGoals();
        void requestReplan(bool random);
        void check_goal();
        bool has_arrived_xy();
        double dist_to_goal_xy();
        void compute_velocity();
        void pathCallback(const potential_exploration::Pose2DArray& msg);
        void loop();
};

potential_driver::potential_driver(){
    goal_th_xy = 0.1;
    inactive_thresh = 5;
    max_lin_speed = 0.5;
    max_ang_speed = 0.75;

    active_goal = 0;
    follow_path = false;
    last_inactive = ros::Time(0);

    position_x = 0;
    position_y = 0;
    position_theta = 0;

    vmsg = geometry_msgs::Twist();

    robot_pos_sub = node_handle.subscribe("/odom", 10, &potential_driver::odometryCallback, this);
    con_path_sub = node_handle.subscribe("/controller_path",10,&potential_driver::pathCallback,this);

    vel_pub = node_handle.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
    rep_pub = node_handle.advertise<std_msgs::Bool>("/mission_replan",1);
}
double potential_driver::dist_to_goal_xy(){
    double delta_xx = std::pow(position_x-goals_x[active_goal],2);
    double delta_yy = std::pow(position_y-goals_y[active_goal],2);
    return std::sqrt(delta_xx+delta_yy);
}
bool potential_driver::has_arrived_xy(){
    return dist_to_goal_xy() < goal_th_xy;
}
void potential_driver::check_goal(){
    if(has_arrived_xy())
        next_goal();
}
void potential_driver::requestReplan(bool random){
    std_msgs::Bool msg = std_msgs::Bool();
    msg.data = random;

    if(random)
        ROS_INFO("Robot possibly blocked,asking for random waypoint...");

    rep_pub.publish(msg);
}
void potential_driver::next_goal(){
    active_goal = active_goal+1;
    if(active_goal == goals_x.size()){
        active_goal = 0;
        follow_path = false;
        clearGoals();
        ROS_INFO("final goal reached...");
    }
}
void potential_driver::clearGoals(){
    vector<double>().swap(goals_x);
    vector<double>().swap(goals_y);
    vector<double>().swap(goals_theta);
}
double potential_driver::angle_wrap(double angle){
    double angle_res;
    if(angle > 0){
        double angle_temp = angle/(2*L_PI);
        int n_p = std::floor(angle_temp);
        angle_res = angle-n_p*(2*L_PI);
        if(angle_res > L_PI)
            angle_res -= (2*L_PI);
    }
    else{
        double angle_temp = angle/(2*L_PI);
        int n_p = std::ceil(angle_temp);
        angle_res = angle-n_p*(2*L_PI);
        if(fabs(angle_res) > L_PI)
            angle_res += (2*L_PI);

    }

    return angle_res;
}
void potential_driver::odometryCallback(const nav_msgs::Odometry& msg){
    mtx.lock();
    position_x = msg.pose.pose.position.x;
    position_y = msg.pose.pose.position.y;

    tf::Quaternion q(msg.pose.pose.orientation.x,\
                     msg.pose.pose.orientation.y,\
                     msg.pose.pose.orientation.z,\
                     msg.pose.pose.orientation.w);

    double roll,pitch,yaw;
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    position_theta = angle_wrap(yaw);
    mtx.unlock();
}

void potential_driver::pathCallback(const potential_exploration::Pose2DArray& msg){
    mtx.lock();
    if(msg.poses.size() == 0){
        follow_path = false;
        if(goals_x.size() != 0)
            last_inactive = ros::Time::now();
        clearGoals();
    }
    else
    {
       last_inactive = ros::Time(0);
       clearGoals();

       for(int i = 0;i < msg.poses.size();i++){
            goals_x.push_back(msg.poses[i].x);
            goals_y.push_back(msg.poses[i].y);
            goals_theta.push_back(msg.poses[i].theta);
       }

       if(msg.poses.size() > 1)
            active_goal = 1;
       else
            active_goal = 0;

       follow_path = true;
    }

    mtx.unlock();
}

void potential_driver::compute_velocity(){
    geometry_msgs::Twist temp_msg = geometry_msgs::Twist();

    if(!follow_path){
        temp_msg.linear.x = vmsg.linear.x*0.4;
        temp_msg.angular.z = vmsg.angular.z*0.4;
    }
    else if(dist_to_goal_xy() > goal_th_xy){
       double angle_to_goal =  atan2(goals_y[active_goal]-position_y,goals_x[active_goal]-position_x);
       double delta_angle_to_goal = angle_wrap(position_theta-angle_to_goal);

       double sign_angle = delta_angle_to_goal/fabs(delta_angle_to_goal);
       temp_msg.angular.z = \
                -sign_angle*std::min(std::min(max_ang_speed,6*fabs(delta_angle_to_goal)),fabs(vmsg.angular.z)*1.1+0.05);

       if(fabs(delta_angle_to_goal) < 0.01){
            temp_msg.linear.x = \
                    std::min(std::min(max_lin_speed,2*dist_to_goal_xy()+0.1),vmsg.linear.x*1.1+0.05);
       }
       else
            temp_msg.linear.x = vmsg.linear.x*0.75;
    }

    vmsg = temp_msg;
}

void potential_driver::loop(){
    mtx.lock();
    if(last_inactive != ros::Time(0)){
        if((ros::Time::now()-last_inactive).toSec() > inactive_thresh){
            requestReplan(true);
            last_inactive = ros::Time(0);
        }
    }

    if(follow_path){
        check_goal();
        compute_velocity();
        vel_pub.publish(vmsg);
    }
    mtx.unlock();
}
int main(int argc, char **argv) {
    ros::init(argc, argv, "potential_driver_node");
    
    potential_driver pd;
    
    ros::Rate r(30);
    while(ros::ok()){
        pd.loop();
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}

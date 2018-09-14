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

double waypoint_threshold = 0.35;
vector<geometry_msgs::Pose2D> wp_history;

typedef struct pose_weight{
   double x;
   double y;
   double w;
   pose_weight(double x_,double y_,double w_)
   {
        x = x_;y = y_;w = w_;
   }
}PoseWeight;

bool cmp(const PoseWeight& a,const PoseWeight& b)
{
    return a.w < b.w;
}

double angle_wrap(double angle){
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

class MissionHandle{
    public:
        boost::mutex mtx;
        bool needs_new_frontier;
        bool fresh_frontiers;

        geometry_msgs::Pose2D current_wp;
        double robot_x,robot_y,robot_theta;

        vector<PoseWeight> frontiers;

        ros::NodeHandle node_handle;

        ros::Subscriber odometry_sub;
        ros::Subscriber potential_map_sub;
        ros::ServiceServer frontier_srv;

        MissionHandle();
        void odometryCallback(const nav_msgs::Odometry& msg);
        void frontierCallback(const nav_msgs::OccupancyGrid& msg);
        bool getFrontier(vector<vector<unsigned int> >& projected_map_,\
                         double resolution_,\
                         double origin_x_,\
                         double origin_y_,\
                         int n_frontier_,\
                         double robot_theta_,\
                         vector<PoseWeight>& v_pose_);
        bool isWaypointInHistory(vector<geometry_msgs::Pose2D>& v,geometry_msgs::Pose2D& c,double th);
        void addWaypointToHistory(vector<geometry_msgs::Pose2D>& v,geometry_msgs::Pose2D& c);
        void proposeWaypoints();
        void updateCurrentWaypoint();
};

MissionHandle::MissionHandle(){
   odometry_sub = node_handle.subscribe("/odom",100,&MissionHandle::odometryCallback,this);
   potential_map_sub = node_handle.subscribe("/potential_map",100,&MissionHandle::frontierCallback,this);
}

void MissionHandle::odometryCallback(const nav_msgs::Odometry& msg){
   mtx.lock();
   robot_x = msg.pose.pose.position.x;
   robot_y = msg.pose.pose.position.y;
   tf::Quaternion q(msg.pose.pose.orientation.x,\
                      msg.pose.pose.orientation.y,\
                      msg.pose.pose.orientation.z,\
                      msg.pose.pose.orientation.w);

   double roll,pitch,yaw;
   tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
     
   robot_theta = angle_wrap(yaw);
   mtx.unlock();
}

bool MissionHandle::getFrontier(vector<vector<unsigned int> >& projected_map_,\
                 double resolution_,\
                 double origin_x_,\
                 double origin_y_,\
                 int n_frontier_,\
                 double robot_theta_,\
                 vector<PoseWeight>& v_pose_){
    int w = projected_map_.size();
    int h = projected_map_[0].size();
    
    ROS_INFO("projected_map_w:%d",w);
    ROS_INFO("projected_map_h:%d",h);
#if 0    
    for(int i = 0;i < w;i++){
        for(int j = 0;j < h;j++){
std::cout << __FILE__ << __LINE__ << std::endl;
            printf("%d",projected_map[i][j]);
std::cout << __FILE__ << __LINE__ << std::endl;

            if(j == w-1)
               printf("\n"); 
        }
    }
#endif
    vector<PoseWeight> v_pose_weight;
    for(int i = 2;i < w-1;i++){
        for(int j = 2;j < h-1;j++){
            if(projected_map_[i][j] < 3)
                continue;

            if(
              ((projected_map_[i][j-1] == 2) && (projected_map_[i-1][j] == 1)) || \
              ((projected_map_[i][j-1] == 1) && (projected_map_[i-1][j] == 2)) || \
              ((projected_map_[i][j+1] == 2) && (projected_map_[i-1][j] == 1)) || \
              ((projected_map_[i][j+1] == 1) && (projected_map_[i-1][j] == 2)) || \
              ((projected_map_[i+1][j] == 2) && (projected_map_[i][j+1] == 1)) || \
              ((projected_map_[i+1][j] == 1) && (projected_map_[i][j+1] == 2)) || \
              ((projected_map_[i+1][j] == 2) && (projected_map_[i][j-1] == 1)) || \
              ((projected_map_[i+1][j] == 1) && (projected_map_[i][j-1] == 2)) || \
              ((projected_map_[i][j-1] == 1) && (projected_map_[i-1][j] == 1)) || \
              ((projected_map_[i][j+1] == 1) && (projected_map_[i-1][j] == 1)) || \
              ((projected_map_[i+1][j] == 1) && (projected_map_[i][j+1] == 1)) || \
              ((projected_map_[i+1][j] == 1) && (projected_map_[i][j-1] == 1))
              )
              {
                double frontier_x = i*resolution_+origin_x_;
                double frontier_y = j*resolution_+origin_y_;

                double angle_to_robot = fabs(angle_wrap(robot_theta_-\
                                        atan2(frontier_y-robot_y,frontier_x-robot_x)));

                double frontier_weight = projected_map_[i][j]*(1+0.8*std::exp(-2.0/angle_to_robot));
                
                PoseWeight pw(frontier_x,frontier_y,frontier_weight);
                v_pose_weight.push_back(pw);
             }
        }
    }
    sort(v_pose_weight.begin(),v_pose_weight.end(),cmp);
    
    if(v_pose_weight.size() > 0){
        ROS_INFO("Find enough frontier:%d",(int)v_pose_weight.size());
        for(int i = 0;i < v_pose_weight.size();i++){
            v_pose_.push_back(PoseWeight(v_pose_weight[i].x,v_pose_weight[i].y,v_pose_weight[i].w));
        }

        return true;
    }
    else{
        return false;
    }

}
	void MissionHandle::frontierCallback(const nav_msgs::OccupancyGrid& msg){
		ROS_INFO("start find frontier...");
        if(msg.info.width * msg.info.height != msg.data.size()){
            ROS_ERROR("Invalid potential map size...");
            return;
        }

        if(msg.info.resolution < 0){
            ROS_ERROR("Invalid potential map resolution");
            return;
        }
        
        int map_height = msg.info.height;
        int map_width = msg.info.width;
        vector<unsigned char> map_data;
        for(int i = 0;i < msg.data.size();i++){
            map_data.push_back(msg.data[i]);
        }

        cv::Mat potential_map(map_height,map_width,CV_8UC1,map_data.data()); 
        vector<vector<unsigned int> > projected_map;

        for(int i = 0;i < potential_map.rows;i++){
            vector<unsigned int> vec_i;
            projected_map.push_back(vec_i);
            for(int j =0 ;j < potential_map.cols;j++){
                projected_map[i].push_back(potential_map.at<unsigned char>(i,j));
            }
        }

        if(getFrontier(projected_map,\
                       msg.info.resolution,\
                       msg.info.origin.position.x,\
                       msg.info.origin.position.y,\
                       0,\
                       robot_theta,\
                       frontiers)){

                for(int i = 0;i < projected_map.size();i++)
                    vector<unsigned int>().swap(projected_map[i]);
                vector<vector<unsigned int>>().swap(projected_map);
                return;
        }
        else{
            for(int i = 0;i < projected_map.size();i++)
                vector<unsigned int>().swap(projected_map[i]);
            vector<vector<unsigned int>>().swap(projected_map);
            return;
        }
	}


bool MissionHandle::isWaypointInHistory(vector<geometry_msgs::Pose2D>& v,geometry_msgs::Pose2D& c,double th){
    bool flag = false;
    if(v.size() <= 0)
        return flag;

    for(int i = 0;i < v.size();i++){
        int delta_x = v[i].x-c.x;
        int delta_y = v[i].y-c.y;

        if(sqrt(delta_x*delta_x+delta_y*delta_y) < th){
            flag = true;
            break;
        }   
    }

    return flag;
}

void MissionHandle::addWaypointToHistory(vector<geometry_msgs::Pose2D>& v,geometry_msgs::Pose2D& c){
    v.push_back(c);
}

void MissionHandle::proposeWaypoints(){
    mtx.lock();
    bool found_waypoint = false;
    int waypoints_tried = 0;
    while((!found_waypoint) && (waypoints_tried < 2*frontiers.size())){
        sort(frontiers.begin(),frontiers.end(),cmp);

        geometry_msgs::Pose2D msg_pose = geometry_msgs::Pose2D();
        msg_pose.x = frontiers[0].x;
        msg_pose.y = frontiers[0].y;
    
        if((waypoints_tried < frontiers.size()) && (isWaypointInHistory(wp_history,msg_pose,waypoint_threshold))){
            frontiers[0].w *= 10;
            waypoints_tried += 1;
            continue;
        }
        
        ros::ServiceClient waypoint_propose_client = \
                           node_handle.serviceClient<potential_exploration::WaypointProposition>("/sweeper/WaypointProposition");
        potential_exploration::WaypointProposition wp_srv;
        wp_srv.request.waypoint.x = msg_pose.x;
        wp_srv.request.waypoint.y = msg_pose.y;

        bool wp_res = waypoint_propose_client.call(wp_srv);
        
        if(wp_res)
            if(wp_srv.response.accepted){
                current_wp.x = msg_pose.x;
                current_wp.y = msg_pose.y;
                needs_new_frontier = false;
                found_waypoint = true;
            }
            else{
                frontiers[0].w *= 10;
                waypoints_tried += 1;
            }
    }
    
    fresh_frontiers = false;
    
    mtx.unlock();
}

void MissionHandle::updateCurrentWaypoint(){
    mtx.lock();

    if(frontiers.size() <= 0){
        mtx.unlock();
        return;
    }

    for(int i = 0;i < frontiers.size();i++){
        if(fabs(current_wp.x-frontiers[i].x) < 0.04 && \
           fabs(current_wp.y-frontiers[i].y) < 0.04){
                if(std::sqrt(std::pow(current_wp.x-robot_x,2)+std::pow(current_wp.y-robot_y,2)) < 0.2){
                    addWaypointToHistory(wp_history,current_wp);
                    break;
                }
                else{
                    mtx.unlock();
                    return;
                }
           }
    }

    fresh_frontiers = false;
    needs_new_frontier = true;
    mtx.unlock();
}
int main(int argc, char **argv) {
    ros::init(argc, argv, "mission_handle_node");

    MissionHandle mh;
   
    ros::Rate r(100);
    while(ros::ok()){
        if(mh.needs_new_frontier && mh.fresh_frontiers)
            mh.proposeWaypoints();
        mh.updateCurrentWaypoint();
        
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}

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

#define M_PI 3.1415926

namespace potential_exploration_ns{
using namespace std;

boost::mutex mtx;
double robot_x, robot_y; 
vector<vector<int>> projected_map; 

struct Pixel{
    int x, y;
    Pixel(int x_in, int y_in){
			x = x_in; y = y_in;
    }
};

void projectedMap(const nav_msgs::OccupancyGrid& msg,double robot_x_,double robot_y_){
    mtx.lock();

    //Resize the projected_map variable for new size
    projected_map.resize(msg.info.width);
    for(int i = 0; i < projected_map.size(); i++)
        projected_map[i].resize(msg.info.height, 0);
    

    //Copy map into 2D array and initialize inflation queue
    //IN: UNKNOWN -1, FREE 0, OCCUPIED 100
    //OUT: UNKNOWN 1, FREE 0, OCCUPIED 2
    deque<Pixel> inflation_queue;
    for(int i = 0; i < projected_map.size(); i++){ //msg.info.width -> columns
        for(int j = 0; j < projected_map[0].size(); j++){//msg.info.height -> rows

            //Obstacle
            if (msg.data[projected_map.size()*j + i] == 100){ 
                projected_map[i][j] = 2;
                //if obstacle add to queue for inflation
                inflation_queue.push_back({i, j});

            //Unknown    
            }else if(msg.data[i +  projected_map.size()*j] == -1){ 
                projected_map[i][j] = 1;

            //Free    
            }else{ 
                projected_map[i][j] = 0;
            }
        }
    }

    //1. INFLATE OBSTACLES
    int inflate = 3;
    deque<Pixel> inflation_queue2;

    for(int n = 0; n < inflate; n++){
        inflation_queue2 = deque<Pixel>();

        while(!inflation_queue.empty()){
            for(int i = std::max(0, inflation_queue.front().x-1); \
                i < std::min(inflation_queue.front().x+2, (int)msg.info.width); i++){
                for(int j = std::max(0, inflation_queue.front().y-1); \
                    j < std::min(inflation_queue.front().y+2, (int)msg.info.height); j++){
                    if(projected_map[i][j] <= 1){
                        projected_map[i][j] = 2;
                        inflation_queue2.push_back(Pixel(i, j));
                    }
                }geometry_msgs/Quaternion getyaw
            }

            inflation_queue.pop_front();
        }

        inflation_queue = inflation_queue2;
    }

    //2. COMPUTE POTENTIAL
    //Put 3 in robot position (have to convert to pixel)
    int robot_pixel_i = round((robot_x_ - msg.info.origin.position.x)/msg.info.resolution);
    int robot_pixel_j = round((robot_y_ - msg.info.origin.position.y)/msg.info.resolution);

    if(robot_pixel_i < 0 || robot_pixel_i > msg.info.width ||
        robot_pixel_j < 0 || robot_pixel_j > msg.info.height){
        ROS_ERROR("Robot outside of projected_map");
        mtx.unlock();
        return;
    }

    projected_map[robot_pixel_i][robot_pixel_j] = 3;

    //Initialize queue with robot position
    deque<Pixel> queue;
    queue.push_back(Pixel(robot_pixel_i, robot_pixel_j));

    while(!queue.empty()){
        for(int i = std::max(0, queue.front().x-1); \
            i < std::min(queue.front().x+2, (int)msg.info.width); i++){
            for(int j = std::max(0, queue.front().y-1); \
                j < std::min(queue.front().y+2, (int)msg.info.height); j++){
                //Free
                if(projected_map[i][j] == 0){
                    projected_map[i][j] = projected_map[queue.front().x][queue.front().y] + 1;
                    queue.push_back(Pixel(i,j));
                }
            }
        }

        queue.pop_front();
    }

    mtx.unlock();
}

double angle_wrap(double angle){
    double angle_temp = angle%(2*M_PI);

    if(angle_temp > M_PI)
        angle_temp -= 2*M_PI;

    return angle_temp;
}

typedef struct pose_weight{
   double x;
   double y;
   double w;
   pose_weight(double x_,double y_,double w_)
   {
        x_ = x;y = y_;w = w_;
   }
}PoseWeight;

bool cmp(const PoseWeight& a,const PoseWeight& b)
{
    return a.w > b.w;
}
bool getFrontier(vector<vector<int> >& projected_map_,\
                 double resolution_,\
                 double origin_x,\
                 double origin_y,\
                 double robot_theta_,\
                 int n_frontier,\
                 vector<geometry_msgs::Pose>& v_pose_){
    mtx.lock();
    double i,j;
    
    int w = projected_map_.size();
    int h = projected_map_[0].size();

    ROS_INFO("projected_map_w:%d",w);
    ROS_INFO("projected_map_h:%d",h);

    vector<PoseWeight> v_pose_weight;

    for(i = 2;i < h-1;i++){
        for(j = 2;j < w-1;j++){
            if(projected_map_[i][j] < 3)
                continue;

            if((projected_map_[i][j-1] == 2 && projected_map_[i-1][j] == 1) || \
              (projected_map_[i][j-1] == 1 && projected_map_[i-1][j] == 2) || \
              (projected_map_[i][j+1] == 2 && projected_map_[i-1][j] == 1) || \
              (projected_map_[i][j+1] == 1 && projected_map_[i-1][j] == 2) || \
              (projected_map_[i+1][j] == 2 && projected_map_[i][j+1] == 1) || \
              (projected_map_[i+1][j] == 1 && projected_map_[i][j+1] == 2) || \
              (projected_map_[i+1][j] == 2 && projected_map_[i][j-1] == 1) || \
              (projected_map_[i+1][j] == 1 && projected_map_[i][j-1] == 2)){
                double frontier_x = j*resolution+origin_x;
                double frontier_y = i*resolution+origin_y;

                double angle_to_robot = abs(angle_wrap(robot_theta_-\
                                        atan2(frontier_y-robot_y,frontier_x-robot_x)));

                double frontier_weight = projected_map_[i][j]*(1+0.8*std::exp(-2.0/angle_to_robot));
                
                v_pose_weight.push_back(PoseWeight(frontier_x,frontier_y,frontier_weight));
              }
        }
    }
    sort(v_pose_weight.begin(),v_pose_weight.end(),cmp);
    
    if(v_pose_weight.size() > n_frontier){
        ROS_INFO("Find enough frontier...");
        for(int i = 0;i < n_frontier;i++){
            geometry_msgs::Pose p;
            p.position.x = v_pose_weight[i].x;
            p.position.y = v_pose_weight[i].y;
            p.position.z = 0;
            v_pose_.push_back(p);
        }
    }
    else
        ROS_ERROR("Find unenough frontier...");

    mtx.unlock();
}
	bool GetNextFrontier(potential_exploration::GetNextFrontier::Request &req,
						 potential_exploration::GetNextFrontier::Response &res){
		std::cout << "start potential exploration..." << std::endl;
        if(req.map.info.width * req.map.info.height != req.map.data.size()){
            ROS_ERROR("Invalid map size...");
            return false;
        }

        if(req.map.info.resolution < 0){
            ROS_ERROR("Invaid map resolution");
            return false;
        }
        
        robot_x = req.start.position.x;
        robot_y = req.start.position.y;

        projectedMap(req.map,robot_x,robot_y);
        
        tf::Quaternion q(req.start.orientation.x,\
                        req.start.orientation.y,\
                        req.start.orientation.z,\
                        req.start.orientation.w);

        double roll,pitch,yaw;
        tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
        
        ROS_INFO("Roll:%f",roll);
        ROS_INFO("Pitch:%f",pitch);
        ROS_INFO("Yaw:%f",yaw);

        vector<geometry_msgs::Pose> v_pose;
        if(getFrontier(projected_map,\
                       req.map.info.resolution,\
                       req.map.info.origin.position.x,\
                       req.map.info.origin.position.y,\
                       req.n_frontier,\
                       yaw,\
                       v_pose)){
                res.goal.poses = v_pose;
                return true;
        }
        else
            return false;
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

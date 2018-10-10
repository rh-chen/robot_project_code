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
#define L_PI 3.1415926

using namespace std;

typedef struct Pixel{
    int x, y;
    Pixel(int x_in, int y_in){
			x = x_in; y = y_in; 
    }
}strPixel;

class potential_map{
public:
    boost::mutex mtx0;
    boost::mutex mtx1;
    boost::mutex mtx2;
    boost::mutex mtx3;
    boost::mutex mtx4;
	
    double map_resolution,map_origin_x,map_origin_y;
    vector<vector<unsigned int> > projected_map;
    ros::NodeHandle node_handle;

    ros::Subscriber robot_pos_sub;
    double robot_x, robot_y,robot_theta;

    ros::Subscriber projected_map_sub;
    ros::Publisher potential_map_pub;

    int robot_pixel_i, robot_pixel_j;
    nav_msgs::OccupancyGrid last_potential_map;
    
    ros::ServiceServer planning_srv;
    ros::Publisher online_traj_pub;

    ros::ServiceServer collision_srv;

    //ros::ServiceServer potential_frontier_srv;

    potential_map();
    void projectedMapCallback(const nav_msgs::OccupancyGrid& msg);
    
    void odometryCallback(const nav_msgs::Odometry& msg);

    bool planPathTo(potential_exploration::PotentialPlanner::Request& request,
                    potential_exploration::PotentialPlanner::Response& response);

    bool checkIfCollisionFree(potential_exploration::CollisionChecker::Request& request,
                              potential_exploration::CollisionChecker::Response& response);

    bool isWalkable(struct Pixel start, struct Pixel end);
    
    void publishRvizPath(const potential_exploration::PotentialPlanner::Response& response);

    /*bool GetNextFrontier(potential_exploration::GetNextFrontier::Request &req,
						 potential_exploration::GetNextFrontier::Response &res);

    bool getFrontier(vector<vector<unsigned int> >& projected_map_,\
                     double resolution_,\
                     double origin_x_,\
                     double origin_y_,\
                     int n_frontier_,\
                     double robot_theta_,\
                     vector<geometry_msgs::Pose>& v_pose_);
    */
    //bool isInHistory(vector<PoseWeight>& v,PoseWeight& c,double th);

    //bool isInHistoryGeometry(vector<geometry_msgs::Pose>& v,geometry_msgs::Pose& c,double th);
    
    double angle_wrap(double angle);
};

potential_map::potential_map()
{
    robot_x = 0; robot_y = 0,robot_theta = 0;

    last_potential_map = nav_msgs::OccupancyGrid();

    robot_pos_sub = node_handle.subscribe("/odom", 10, &potential_map::odometryCallback, this);

    projected_map_sub = node_handle.subscribe("/map", 1, &potential_map::projectedMapCallback, this);

    potential_map_pub = node_handle.advertise<nav_msgs::OccupancyGrid>("/potential_map", 1);

    planning_srv = node_handle.advertiseService("/sweeper/PotentialPlanner", &potential_map::planPathTo, this);
    
    online_traj_pub = node_handle.advertise<visualization_msgs::Marker> ("/sweeper/solution_path", 3, true);

    collision_srv = node_handle.advertiseService("/sweeper/CollisionChecker", &potential_map::checkIfCollisionFree, this);

    //potential_frontier_srv = node_handle.advertiseService("/sweeper/potential_frontier",&potential_map::GetNextFrontier,this);

}

void potential_map::odometryCallback(const nav_msgs::Odometry& msg){
     mtx0.lock();
     robot_x = msg.pose.pose.position.x;
     robot_y = msg.pose.pose.position.y;

     tf::Quaternion q(msg.pose.pose.orientation.x,\
                      msg.pose.pose.orientation.y,\
                      msg.pose.pose.orientation.z,\
                      msg.pose.pose.orientation.w);

     double roll,pitch,yaw;
     tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
     
     robot_theta = angle_wrap(yaw);
     mtx0.unlock();
}

void potential_map::projectedMapCallback(const nav_msgs::OccupancyGrid& msg){
    mtx1.lock();
//std::cout << __FILE__ << __LINE__ << std::endl;
    map_resolution = msg.info.resolution;
    map_origin_x = msg.info.origin.position.x;
    map_origin_y = msg.info.origin.position.y;

    ROS_INFO("map_width:%d",(int)msg.info.width);
    ROS_INFO("map_height:%d",(int)msg.info.height);

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
                inflation_queue.push_back(Pixel(i, j));

            //Unknown    
            }else if(msg.data[projected_map.size()*j + i] == -1){ 
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
                }
            }

            inflation_queue.pop_front();
        }

        inflation_queue = inflation_queue2;
    }

    //2. COMPUTE POTENTIAL
    //Put 3 in robot position (have to convert to pixel)
    robot_pixel_i = round((robot_x - msg.info.origin.position.x)/msg.info.resolution);
    robot_pixel_j = round((robot_y - msg.info.origin.position.y)/msg.info.resolution);

    if(robot_pixel_i < 0 || robot_pixel_i > msg.info.width ||
        robot_pixel_j < 0 || robot_pixel_j > msg.info.height){
        ROS_ERROR("Robot outside of projected_map");
        mtx1.unlock();
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

    last_potential_map.header = msg.header;
    last_potential_map.info = msg.info;
    vector<signed char> vec_data;
    for(int j = 0; j < projected_map[0].size(); j++)
        for(int i = 0; i < projected_map.size(); i++)
            vec_data.push_back(projected_map[i][j]);

    last_potential_map.data = vec_data;

    potential_map_pub.publish(last_potential_map);  
    /*for(int i = 0;i < projected_map.size();i++)
	vector<unsigned int>().swap(projected_map[i]);
    vector<vector<unsigned int>>().swap(projected_map);
    */
    vector<signed char>().swap(vec_data);    
//std::cout << __FILE__ << __LINE__ << std::endl;
    mtx1.unlock();
}
bool potential_map::planPathTo(potential_exploration::PotentialPlanner::Request& request, 
                               potential_exploration::PotentialPlanner::Response& response)
{
    mtx2.lock();
//std::cout << __FILE__ << __LINE__ << std::endl;
    float goal_x = request.goal_state_x,
          goal_y = request.goal_state_y,
          map_origin_x = last_potential_map.info.origin.position.x,
          map_origin_y = last_potential_map.info.origin.position.y,
          map_resolution = last_potential_map.info.resolution;

    int goal_px_x = round((goal_x - map_origin_x)/map_resolution),
        goal_px_y = round((goal_y - map_origin_y)/map_resolution);

    ROS_INFO("goal_px_x:%d",goal_px_x);
    ROS_INFO("goal_px_y:%d",goal_px_y);
if(goal_px_x >= last_potential_map.info.width || goal_px_x < 0){
	ROS_WARN("goal is outside map...");
	mtx2.unlock();
	return false;
    }
if(goal_px_y >= last_potential_map.info.height || goal_px_y < 0){
	ROS_WARN("goal is outside map...");
	mtx2.unlock();
	return false;
    }
std::cout << __FILE__ << __LINE__ << std::endl;
    if(projected_map[goal_px_x][goal_px_y] == 2){
        ROS_ERROR("Goal is marked as occupied.");
        mtx2.unlock();
        return false;
    }
std::cout << __FILE__ << __LINE__ << std::endl;
    struct Pixel robot_cell = Pixel(robot_pixel_i, robot_pixel_j);
    struct Pixel current_cell = Pixel(goal_px_x, goal_px_y);
    vector<Pixel> pixel_path = vector<Pixel>();

    int watchdog_count = 0;
    int watchdog_max = 1500;

    while(current_cell.x != robot_cell.x || current_cell.y != robot_cell.y){
        //Add current_cell to path
        pixel_path.push_back(current_cell);

//std::cout << __FILE__ << __LINE__ << std::endl;
        //Init neighbourhood search
        float min_potential = (float)INT_MAX;
        float compare_potential = (float)INT_MAX; //Used for weighting the diagonals
        struct Pixel min_pixel = Pixel(-1, -1);

        //Prepare for loop limits
        int i_start = std::max(0, current_cell.x-1);
        int i_end = std::min(current_cell.x+2, (int)last_potential_map.info.width);

        int j_start = std::max(0, current_cell.y-1);
        int j_end = std::min(current_cell.y+2, (int)last_potential_map.info.height);

//std::cout << __FILE__ << __LINE__ << std::endl;
        for(int i = i_start; i < i_end; i++){
            for(int j = j_start; j < j_end; j++){
                //Can be unknown or obstacle (1,2) or potential >= 3
                if(projected_map[i][j] < 3) continue;

                compare_potential = projected_map[i][j];
                //Diagonal check (increase potential to make them less attractive)
                if( (abs(i-current_cell.x) + abs(j-current_cell.y)) == 2 )
                    compare_potential += 0.5;
                
                //Comparison
                if(compare_potential <= min_potential){
                    min_pixel.x = i; 
                    min_pixel.y = j;
                    min_potential = compare_potential;
                }
            }
        }
        //Update cell to add to path
        current_cell = min_pixel;

std::cout << __FILE__ << __LINE__ << std::endl;
        //Watchdog anti-blocker
        if(watchdog_count++ > watchdog_max){
            ROS_WARN("No path found, max iter reached");
            response.poses.clear();
 
std::cout << __FILE__ << __LINE__ << std::endl;
            mtx2.unlock();
            return false;
        }
    }


std::cout << __FILE__ << __LINE__ << std::endl;
    //GREEDY smoothing
    vector<Pixel> pixel_path_smooth = vector<Pixel>();
    pixel_path_smooth.push_back(pixel_path.back());

    int segment_end_idx = pixel_path.size()-1;
    while(segment_end_idx != 0){
        for(int i = 0; i < segment_end_idx; i++){
            if(isWalkable(pixel_path.at(i), 
                          pixel_path.at(segment_end_idx)))
            {   
                //Add new point to smooth path
                pixel_path_smooth.push_back(pixel_path.at(i));

                //Reset search for segment until this one
                segment_end_idx = i;

                //Get out of loop
                break;
            }
        }
    }
    // Reverse path to get start to goal path
    std::reverse(pixel_path_smooth.begin(), pixel_path_smooth.end());

//std::cout << __FILE__ << __LINE__ << std::endl;
    //Convert from pixel path (i,j) to pose path (x,y) for driver
    geometry_msgs::Pose2D pose;
    for(int i = 0; i < pixel_path_smooth.size(); i ++){
        pose = geometry_msgs::Pose2D();
        pose.x = pixel_path_smooth.at(i).x * map_resolution + map_origin_x;
        pose.y = pixel_path_smooth.at(i).y * map_resolution + map_origin_y;
        response.poses.insert(response.poses.begin(), pose);
    }

    //publishRvizPath(response);

//std::cout << __FILE__ << __LINE__ << std::endl;
    mtx2.unlock();
    return true;
}

//############################################
// Collision checking

bool potential_map::checkIfCollisionFree(potential_exploration::CollisionChecker::Request& request, 
                                         potential_exploration::CollisionChecker::Response& response)
{
    mtx3.lock();
std::cout << __FILE__ << __LINE__ << std::endl;
    float map_origin_x = last_potential_map.info.origin.position.x,
          map_origin_y = last_potential_map.info.origin.position.y,
          map_resolution = last_potential_map.info.resolution;

    response.path_safe = true;

    struct Pixel  start_px(-1, -1), end_px(-1,-1);
    for(int i = 0; i < request.poses.size()-1; i++){
        //Check each point in the way
        start_px.x = round((request.poses.at(i).x - map_origin_x)/map_resolution);
        start_px.y = round((request.poses.at(i).y - map_origin_y)/map_resolution);

        end_px.x = round((request.poses.at(i+1).x - map_origin_x)/map_resolution);
        end_px.y = round((request.poses.at(i+1).y - map_origin_y)/map_resolution);

        if(!isWalkable(start_px, end_px)) {
            response.path_safe = false; 
            break;
        }
    }

std::cout << __FILE__ << __LINE__ << std::endl;
    mtx3.unlock();
    return true;
}

bool potential_map::isWalkable(struct Pixel start, struct Pixel end){
    //Checks collision-free straight line from start to end

//std::cout << __FILE__ << __LINE__ << std::endl;
    //Get vector between start and end
    float path_x = end.x - start.x;
    float path_y = end.y - start.y;
    float path_length = std::sqrt(path_x*path_x + path_y*path_y);

    //Prepare step increase
    float step_length = 0.2;
    float delta_x = step_length*(path_x/path_length); 
    float delta_y = step_length*(path_y/path_length);

    //Prepare checking loop
    float check_point_x = start.x;
    float check_point_y = start.y;
    for(float i = 0; i < path_length; i += step_length){
        //Obstacle == 2 in projected_map; <= 2 for obstacle + unknown space
        if(projected_map.at(std::round(check_point_x)).at(std::round(check_point_y)) == 2) 
            return false;

        check_point_x += delta_x;
        check_point_y += delta_y;
    }

//std::cout << __FILE__ << __LINE__ << std::endl;
    return true;
}

void potential_map::publishRvizPath(const potential_exploration::PotentialPlanner::Response& response){
    //Send path to rviz
    visualization_msgs::Marker m = visualization_msgs::Marker();

    m.header.frame_id = "/odom";
    m.header.stamp = ros::Time::now();
    m.action = visualization_msgs::Marker::ADD;
    m.pose.orientation.w = 1.0;
    m.id = 0;
    m.type = visualization_msgs::Marker::LINE_LIST;
    m.scale.x = 0.02;

    m.color.g = m.color.a = 1.0;  
    m.ns = "potential_path";  

    geometry_msgs::Point p;
    for(int i = 1; i < response.poses.size(); i++){
        //Build pose for marker display
        p = geometry_msgs::Point();
        p.x = response.poses[i].x;
        p.y = response.poses[i].y;
        p.z = 0.2;
        m.points.push_back(p);

        p = geometry_msgs::Point();
        p.x = response.poses[i-1].x;
        p.y = response.poses[i-1].y;
        p.z = 0.2;
        m.points.push_back(p);
    }

    online_traj_pub.publish(m);
}
/*bool potential_map::isInHistory(vector<PoseWeight>& v,PoseWeight& c,double th){
    bool flag = false;
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

bool potential_map::isInHistoryGeometry(vector<geometry_msgs::Pose>& v,geometry_msgs::Pose& c,double th){
    bool flag = false;
    for(int i = 0;i < v.size();i++){
        int delta_x = v[i].position.x-c.position.x;
        int delta_y = v[i].position.y-c.position.y;

        if(sqrt(delta_x*delta_x+delta_y*delta_y) < th){
            flag = true;
            break;
        }   
    }

    return flag;
}*/
double potential_map::angle_wrap(double angle){
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

/*
bool potential_map::getFrontier(vector<vector<unsigned int> >& projected_map_,\
                 double resolution_,\
                 double origin_x_,\
                 double origin_y_,\
                 int n_frontier_,\
                 double robot_theta_,\
                 vector<geometry_msgs::Pose>& v_pose_){
    int w = projected_map_.size();
    int h = projected_map_[0].size();
    
    ROS_INFO("projected_map_w:%d",w);
    ROS_INFO("projected_map_h:%d",h);

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
              ){
                double frontier_x = i*resolution_+origin_x_;
                double frontier_y = j*resolution_+origin_y_;

                double angle_to_robot = fabs(angle_wrap(robot_theta_-\
                                        atan2(frontier_y-robot_y,frontier_x-robot_x)));

                double frontier_weight = projected_map_[i][j]*(1+0.8*std::exp(-2.0/angle_to_robot));
                
                PoseWeight pw(frontier_x,frontier_y,frontier_weight);
                if(!isInHistory(v_pose_weight,pw,threshold_frontier))
                    v_pose_weight.push_back(pw);
              }
        }
    }
    sort(v_pose_weight.begin(),v_pose_weight.end(),cmp);
    
    if(v_pose_weight.size() > 0){
        ROS_INFO("Find enough frontier...");
        ROS_INFO("Frontier_num:%d",(int)v_pose_weight.size());

        for(int i = 0;i < v_pose_weight.size();i++){
            geometry_msgs::Pose p;
            p.position.x = v_pose_weight[i].x;
            p.position.y = v_pose_weight[i].y;
            p.position.z = 0;
            v_pose_.push_back(p);
        }
        return true;
    }
    else{
        ROS_ERROR("Find unenough frontier...");
        ROS_INFO("Frontier_num:%d",(int)v_pose_weight.size());
        return false;
    }

}
	bool potential_map::GetNextFrontier(potential_exploration::GetNextFrontier::Request &req,
						                potential_exploration::GetNextFrontier::Response &res){
		ROS_INFO("start potential exploration...");

        vector<geometry_msgs::Pose> v_pose;
        if(getFrontier(projected_map,\
                       map_resolution,\
                       map_origin_x,\
                       map_origin_y,\
                       req.n_frontier,\
                       robot_theta,\
                       v_pose)){
                res.goal.poses = v_pose;

                for(int i = 0;i < projected_map.size();i++)
                    vector<unsigned int>().swap(projected_map[i]);
                vector<vector<unsigned int>>().swap(projected_map);
                return true;
        }
        else{
            res.goal.poses = v_pose;
            for(int i = 0;i < projected_map.size();i++)
                vector<unsigned int>().swap(projected_map[i]);
            vector<vector<unsigned int>>().swap(projected_map);
            return true;
        }
	}
*/
int main(int argc, char **argv) {
    ros::init(argc, argv, "potential_exploration_node");

    potential_map pm;

    ros::spin();

    return (0);
}

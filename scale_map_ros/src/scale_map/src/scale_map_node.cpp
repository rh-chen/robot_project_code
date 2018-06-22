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
#include "scale_map/ScaleMapData.h"

#define SCALE_FACTOR 4

namespace scale_map{

using namespace __gnu_cxx; 
using namespace std;
using namespace cv;

enum CellType{
		Unexplored = -1,
		Empty = 0,
		Obstacle = 100
};

void delete_element(vector<cv::Point>& vec,cv::Point& p)
{
	std::cout << "before erase vec size:" << vec.size() << std::endl;
	for(vector<cv::Point>::iterator iter = vec.begin();iter != vec.end();)
	{
		if(((*iter).x == p.x) && ((*iter).y == p.y))
			iter = vec.erase(iter);
		else
			iter++;
	}

	std::cout << "after erase vec size:" << vec.size() << std::endl;
}

bool ScaleMapService(
    scale_map::ScaleMapData::Request &req,     
    scale_map::ScaleMapData::Response &res) { 
  if (req.erosion_radius < 0) {
    ROS_ERROR("erosion_radius < 0");
    return false;
  }
  if (req.robot_radius < 0) {
    ROS_ERROR("robot_radius < 0");
    return false;
  }

	
  /*for (int i = 0; i < req.map.data.size(); ++i) {
    if (-1 == req.map.data[i]) {  
      req.map.data[i] = 100;
    }
  }*/

	std::cout << "map_height:" << req.map.info.height << std::endl;
	std::cout << "map_width:" << req.map.info.width << std::endl;

  cv::Mat map(req.map.info.height, req.map.info.width, CV_8SC1, req.map.data.data());
	
  ROS_INFO("start to scale map.");

	float ng_resolution = req.map.info.resolution * SCALE_FACTOR;

	int ng_width = req.map.info.width/SCALE_FACTOR;
	int ng_height = req.map.info.height/SCALE_FACTOR;
std::cout << "ng_height:" << ng_height << std::endl;
std::cout << "ng_width:" << ng_width << std::endl;
	int ng_row;
	int ng_col;
	int temp_ng_row;
	int temp_ng_col;

	bool skip;

	vector<cv::Point> obstacles;
	vector<cv::Point> empty;

	vector<vector<int8_t> > ng_data;
	vector<int8_t> ng_data_1d;

	char currentCellValue;
	char ng_oldCellValue;

	bool cacheObstacleCells = true;
	bool cacheEmptyCells = true;

//std::cout << __FILE__ << __LINE__ << std::endl;
	ng_row = -1;
	for(int i = 0;i < req.map.info.height;i++){
		temp_ng_row = i/SCALE_FACTOR;

		if(ng_row != temp_ng_row){
			ng_row = temp_ng_row;
			vector<int8_t> vec_i;
			ng_data.push_back(vec_i);
		}

//std::cout << __FILE__ << __LINE__ << std::endl;
		ng_col = -1;
		for(int j = 0;j < req.map.info.width;j++){
			temp_ng_col = j/SCALE_FACTOR;

			if(ng_col != temp_ng_col){
				ng_col = temp_ng_col;
				skip = false;

//std::cout << __FILE__ << __LINE__ << std::endl;

				ng_data[ng_row].push_back(-2);
//std::cout << __FILE__ << __LINE__ << std::endl;
			}
			
//std::cout << __FILE__ << __LINE__ << std::endl;
			int num_count = count(obstacles.begin(),obstacles.end(),cv::Point(ng_col,ng_row));
			//ROS_INFO("num_count:%d",num_count);
			if(num_count > 0)
				skip = true;

			if(skip)
				continue;
			
//std::cout << __FILE__ << __LINE__ << std::endl;
			currentCellValue = map.at<char>(i,j);
			ng_oldCellValue = ng_data[ng_row][ng_col];

			if(currentCellValue == CellType::Obstacle){
				ng_data[ng_row][ng_col] = CellType::Obstacle;
				if(cacheObstacleCells)
					obstacles.push_back(cv::Point(ng_col,ng_row));
				if(cacheEmptyCells && (ng_oldCellValue == CellType::Empty))
				{
					cv::Point temp_point(ng_col,ng_row);
					
//std::cout << __FILE__ << __LINE__ << std::endl;
					delete_element(empty,temp_point);
					//empty.remove(cv::Point(ng_col,ng_row));
//std::cout << __FILE__ << __LINE__ << std::endl;
				}
			}
			else if(currentCellValue == CellType::Unexplored){
				if(ng_oldCellValue != CellType::Obstacle){
					ng_data[ng_row][ng_col] = CellType::Unexplored;
					if(cacheEmptyCells && (ng_oldCellValue == CellType::Empty))
					{
						cv::Point temp_point(ng_col,ng_row);
//std::cout << __FILE__ << __LINE__ << std::endl;
						delete_element(empty,temp_point);
						//empty.remove(cv::Point(ng_col,ng_row));
//std::cout << __FILE__ << __LINE__ << std::endl;
					}
				}
			}
			else{
				if((ng_oldCellValue != CellType::Obstacle) && (ng_oldCellValue != CellType::Unexplored)){
					ng_data[ng_row][ng_col] = CellType::Empty;
					if(cacheEmptyCells)
						empty.push_back(cv::Point(ng_col,ng_row));
				}
			}
		}
	}

//std::cout << __FILE__ << __LINE__ << std::endl;
std::cout << "ng_data.size:" << ng_data.size() << std::endl;
std::cout << "ng_data[i].size:" << ng_data[0].size() << std::endl;

	for(int i = 0;i < ng_height;i++){
		for(int j = 0;j < ng_width;j++)
		{
			ng_data_1d.push_back(ng_data[i][j]);
		}
	}

//std::cout << __FILE__ << __LINE__ << std::endl;

	res.map.info.height = ng_height;
	res.map.info.width = ng_width;
	res.map.info.resolution = ng_resolution;

	res.map.data = ng_data_1d;
	res.map.header.frame_id = req.map.header.frame_id;

	res.map.info.origin.position.x = req.map.info.origin.position.x;
	res.map.info.origin.position.y = req.map.info.origin.position.y;

  return true;
}

}

int main(int argc, char **argv) {
  ros::init(argc, argv, "scale_map_node");

  ros::NodeHandle private_nh("~");

  ros::ServiceServer scale_map_srv = private_nh.advertiseService("/sweeper/scale_map_srv",scale_map::ScaleMapService);

  ROS_INFO("scale map service active.");

  ros::spin();

  return 0;
}

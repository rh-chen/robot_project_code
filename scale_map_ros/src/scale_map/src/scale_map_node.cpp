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

#define SCALE_FACTOR 6

namespace scale_map{

using namespace __gnu_cxx; 
using namespace std;
using namespace cv;

enum CellType{
		Unexplored = -1,
		Empty = 0,
		Obstacle = 100
};

typedef struct point_str{
	int x;
	int y;
	
	point_str(){};
	point_str(int x_,int y_){x = x_;y = y_; }
}PointStr;

/*class pointCmp{
	public:
	bool operator()(PointStr const &p_a,PointStr const &p_b)const
	{
			if((p_a.x*p_a.y) == (p_b.x*p_b.y))
				return (p_a.x < p_b.x?true:false);
			else
				return ((p_a.x*p_a.y) < (p_b.x*p_b.y)?true:false);
	}
};*/

bool operator<(PointStr const &p_a,PointStr const &p_b)
{
		if((p_a.x*p_a.y) == (p_b.x*p_b.y))
			return (p_a.x < p_b.x?true:false);
		else
			return ((p_a.x*p_a.y) < (p_b.x*p_b.y)?true:false);
}

void delete_element(map<PointStr,int>& map_data,PointStr& p)
{
	map<PointStr,int>::iterator iter;
	iter = map_data.find(p);

	map_data.erase(iter);

	//std::cout << "after erase vec size:" << vec.size() << std::endl;
}
map<PointStr,int> obstacles;
map<PointStr,int> empty;
map<PointStr,int>::iterator iter_ob;
map<PointStr,int>::iterator iter_em;
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


    if(SCALE_FACTOR == 1){
        res.map = req.map;
        return true;
    }
	std::cout << "map_height:" << req.map.info.height << std::endl;
	std::cout << "map_width:" << req.map.info.width << std::endl;

    cv::Mat map(req.map.info.height, req.map.info.width, CV_8SC1, req.map.data.data());
    ROS_INFO("start to scale map.");

    /*int map_start_col = 0;
    int map_end_col = map.cols;
    bool map_start_flag = false;
    bool map_end_flag = false;

    //map_start_col
    for(int j = 0;j < map.cols;j++){
        if(!map_start_flag){
        for(int i = 0;i < map.rows;i++){
            if(map.at<signed char>(i,j) == CellType::Empty)
            {
                map_start_col = j;
                map_start_flag = true;
                break;
            }
        }
        }
        else
            break;
    }
    //map_end_col
    for(int j = map.cols-1;j >= 0;j--){
        if(!map_end_flag){
        for(int i = map.rows-1;i >= 0;i--){
            if(map.at<signed char>(i,j) == CellType::Empty)
            {
                map_end_col = j;
                map_end_flag = true;
                break;
            }
        }
        }
        else
            break;
    }
    
    ROS_INFO("map_start_col:%d",map_start_col);
    ROS_INFO("map_end_col:%d",map_end_col);
    
    int SCALE_FACTOR;
    
    int col_delta_6 = (map_end_col-map_start_col+1)%6;
    int col_delta_5 = (map_end_col-map_start_col+1)%5;
    int col_delta_7 = (map_end_col-map_start_col+1)%7;

    ROS_INFO("col_delta_6:%d",col_delta_6);
    ROS_INFO("col_delta_5:%d",col_delta_5);
    ROS_INFO("col_delta_7:%d",col_delta_7);

    if(std::min(std::min(col_delta_5,col_delta_6),col_delta_7) == col_delta_6)
        SCALE_FACTOR = 6;
    else if(std::min(std::min(col_delta_5,col_delta_6),col_delta_7) == col_delta_5)
        SCALE_FACTOR = 5;
    else
        SCALE_FACTOR = 7;
    */

	float ng_resolution = req.map.info.resolution * SCALE_FACTOR;

    int ng_width,ng_height;
    if((req.map.info.width%SCALE_FACTOR) == 0)
        ng_width = req.map.info.width/SCALE_FACTOR;
    else
        ng_width = (req.map.info.width+(SCALE_FACTOR-req.map.info.width%SCALE_FACTOR))/SCALE_FACTOR;

    if((req.map.info.height%SCALE_FACTOR) == 0)
        ng_height = req.map.info.height/SCALE_FACTOR;
    else
        ng_height = (req.map.info.height+(SCALE_FACTOR-req.map.info.height%SCALE_FACTOR))/SCALE_FACTOR;
    
	std::cout << "ng_height:" << ng_height << std::endl;
	std::cout << "ng_width:" << ng_width << std::endl;
	int ng_row;
	int ng_col;
	int temp_ng_row;
	int temp_ng_col;

	bool skip;

	//map<PointStr,int> obstacles;
	uint64 obstacle_count = 0;
	//map<PointStr,int> empty;
	uint64 empty_count = 0;

	vector<vector<signed char> > ng_data;
	vector<int8_t> ng_data_1d;

	signed char currentCellValue;
	signed char ng_oldCellValue;
    
    double scale_grid = 0.05;

	bool cacheObstacleCells = true;
	bool cacheEmptyCells = true;

    vector<vector<unsigned int> > ng_data_temp;
    ng_data_temp.resize(ng_height);
    for(int i = 0;i < ng_data_temp.size();i++)
        ng_data_temp[i].resize(ng_width,0);


    for(int i = 0;i < req.map.info.height;i++){
        for(int j = 0;j < req.map.info.width;j++){
            int temp_i = i/SCALE_FACTOR;
            int temp_j = j/SCALE_FACTOR;

            if((map.at<signed char>(i,j) == -1) || (map.at<signed char>(i,j) == 100))
                ng_data_temp[temp_i][temp_j] += 1;
        }
    }
//std::cout << __FILE__ << __LINE__ << std::endl;
	ng_row = -1;
	for(int i = 0;i < req.map.info.height;i++){
		temp_ng_row = i/SCALE_FACTOR;
        
        if(temp_ng_row == ng_height)
            continue;

		if(ng_row != temp_ng_row){
			ng_row = temp_ng_row;
			vector<signed char> vec_i;
			ng_data.push_back(vec_i);
		}

//std::cout << __FILE__ << __LINE__ << std::endl;
		ng_col = -1;
		for(int j = 0;j < req.map.info.width;j++){
			temp_ng_col = j/SCALE_FACTOR;
            
            if(temp_ng_col == ng_width)
                continue;

			if(ng_col != temp_ng_col){
				ng_col = temp_ng_col;
				skip = false;

//std::cout << __FILE__ << __LINE__ << std::endl;

				ng_data[ng_row].push_back(-2);
//std::cout << __FILE__ << __LINE__ << std::endl;
			}
			
//std::cout << __FILE__ << __LINE__ << std::endl;
			//int num_count = count(obstacles.begin(),obstacles.end(),cv::Point(ng_col,ng_row));
			//ROS_INFO("num_count:%d",num_count);
			PointStr point_data(ng_col,ng_row);
			if(obstacles.count(point_data))
				skip = true;

			if(skip)
				continue;
			
//std::cout << __FILE__ << __LINE__ << std::endl;
			currentCellValue = map.at<signed char>(i,j);
			ng_oldCellValue = ng_data[ng_row][ng_col];
            
			if(currentCellValue == CellType::Obstacle){
                //ROS_INFO("ng_data_temp:%d",ng_data_temp[ng_row][ng_col]);
                if(ng_data_temp[ng_row][ng_col] >= (int)floor(SCALE_FACTOR*SCALE_FACTOR*scale_grid)){
				    ng_data[ng_row][ng_col] = CellType::Obstacle;
                    if(cacheObstacleCells){
					    PointStr point_temp(ng_col,ng_row);
					    obstacles.insert(pair<PointStr,int>(point_temp,obstacle_count++));
				    }
                    
                    if(cacheEmptyCells && (ng_oldCellValue == CellType::Empty))
				    {
					    PointStr point_temp(ng_col,ng_row);
					
					    delete_element(empty,point_temp);
				    }
                }
                else{
                    ng_data[ng_row][ng_col] = CellType::Empty;
                    if(cacheEmptyCells){
						PointStr point_temp(ng_col,ng_row);
						empty.insert(pair<PointStr,int>(point_temp,empty_count++));
					}
                }
				/*if(cacheObstacleCells){
					PointStr point_temp(ng_col,ng_row);
					obstacles.insert(pair<PointStr,int>(point_temp,obstacle_count++));
				}*/
				/*if(cacheEmptyCells && (ng_oldCellValue == CellType::Empty))
				{
					PointStr point_temp(ng_col,ng_row);
					
//std::cout << __FILE__ << __LINE__ << std::endl;
					delete_element(empty,point_temp);
					//empty.remove(cv::Point(ng_col,ng_row));
//std::cout << __FILE__ << __LINE__ << std::endl;
				}*/
			}
			else if(currentCellValue == CellType::Unexplored){
                //ROS_INFO("ng_data_temp:%d",ng_data_temp[ng_row][ng_col]);
                if(ng_data_temp[ng_row][ng_col] >= (int)floor(SCALE_FACTOR*SCALE_FACTOR*scale_grid)){
				    if(ng_oldCellValue != CellType::Obstacle){
					    ng_data[ng_row][ng_col] = CellType::Unexplored;
					    if(cacheEmptyCells && (ng_oldCellValue == CellType::Empty))
					    {
						    PointStr point_temp(ng_col,ng_row);
//std::cout << __FILE__ << __LINE__ << std::endl;
						    delete_element(empty,point_temp);
						//empty.remove(cv::Point(ng_col,ng_row));
//std::cout << __FILE__ << __LINE__ << std::endl;
					    }
				    }
                }
                else{
                    ng_data[ng_row][ng_col] = CellType::Empty;
                    if(cacheEmptyCells){
						PointStr point_temp(ng_col,ng_row);
						empty.insert(pair<PointStr,int>(point_temp,empty_count++));
					}
                }
			}
			else{
				if((ng_oldCellValue != CellType::Obstacle) && (ng_oldCellValue != CellType::Unexplored)){
					ng_data[ng_row][ng_col] = CellType::Empty;
					if(cacheEmptyCells){
						PointStr point_temp(ng_col,ng_row);
						empty.insert(pair<PointStr,int>(point_temp,empty_count++));
					}
				}
			}
		}
	}



#if 0
//expand obstacles
int erosion_radius = req.erosion_radius;
for(iter_ob = obstacles.begin();iter_ob != obstacles.end();iter_ob++){
	PointStr temp = iter_ob->first;

	int x = temp.x;
	int y = temp.y;

	int x_min = (x-erosion_radius) >= 0?(x-erosion_radius):0;
	int y_min = (y-erosion_radius) >= 0?(y-erosion_radius):0;

	int x_max = (x+erosion_radius) <= (ng_width-1)?(x+erosion_radius):(ng_width-1);
	int y_max = (y+erosion_radius) <= (ng_height-1)?(y+erosion_radius):(ng_height-1);

	for(int i = y_min;i <= y_max;i++){
		for(int j = x_min;j <= x_max;j++)
		{
			if(ng_data[i][j] == CellType::Empty)
				ng_data[i][j] = CellType::Obstacle;
		}
	}
}
#endif
//std::cout << __FILE__ << __LINE__ << std::endl;
std::cout << "ng_data.size:" << ng_data.size() << std::endl;
std::cout << "ng_data[i].size:" << ng_data[0].size() << std::endl;

	for(int i = 0;i < req.map.info.height/SCALE_FACTOR;i++){
		for(int j = 0;j < req.map.info.width/SCALE_FACTOR;j++)
		{
            if(ng_data[i][j] == -2)
			    ng_data_1d.push_back(100);
			else
                ng_data_1d.push_back(ng_data[i][j]);
		}
	}

//std::cout << __FILE__ << __LINE__ << std::endl;

	res.map.info.height = req.map.info.height/SCALE_FACTOR;
	res.map.info.width = req.map.info.width/SCALE_FACTOR;
	res.map.info.resolution = ng_resolution;

	res.map.data = ng_data_1d;
	res.map.header.frame_id = req.map.header.frame_id;

	res.map.info.origin.position.x = req.map.info.origin.position.x;
	res.map.info.origin.position.y = req.map.info.origin.position.y;

	obstacles.clear();
	empty.clear();
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

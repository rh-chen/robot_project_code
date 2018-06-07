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

namespace darp_ros {

using namespace std;
using namespace cv;

boost::shared_ptr<boost::thread> darp_thread;

cv::Mat environment_grid_;
cv::Mat binary_grid_;
cv::Mat label_2d_;

class DARP
{
	public:
	
	int rows,cols;
	int nr;
	int obs;

	vector<bool> ConnectedRobotRegions;
	cv::Mat robotBinary;
	cv::Mat A;
	cv::Mat GridEnv;
	
	DARP(int rows_,int cols_,cv::Mat& src_)
	{
		rows = rows_;
		cols = cols_;
		
		deepCopyMatrix(src_,GridEnv);
		
		printf("GridEnv.rows:%d\n",GridEnv.rows);
		printf("GridEnv.cols:%d\n",GridEnv.cols);

		for(int i = 0;i < GridEnv.rows;i++)
			for(int j = 0;j < GridEnv.cols;j++)
			{
				std::cout << GridEnv.at<int>(i,j) << " ";
				if(j == GridEnv.cols-1)
					printf("\n");
			}
		nr = 0;
		obs = 0;
	}

	void deepCopyMatrix(cv::Mat& data_,cv::Mat& dst_)
	{data_.copyTo(dst_);}
};


class ConnectComponent
{
	public:
	ConnectComponent(int rows_,int cols_)
	{
		printf("construct ConnectComponent object......\n");
		next_label = 1;
		rows = rows_;
		cols = cols_;
		MAX_LABELS = rows_*cols_;
	}

	int MAX_LABELS;
	int next_label;
	cv::Mat BinaryRobot,BinaryNonRobot;
	int rows,cols;


	int getMaxLabel() {return next_label;}

	void compactLabeling(cv::Mat& data_,cv::Mat& label_2d_,int rows,int cols,bool zeroAsBg)
	{
		vector<int> image;
		transform2Dto1D(data_,image,rows,cols);

		printf("transform 2 Dimension to 1 Dimension......\n");
		for(int i = 0;i < image.size();i++)
			std::cout << image[i] << " ";
		std::cout << std::endl;
		
std::cout << __FILE__ << __LINE__ << std::endl;
		vector<int> label(MAX_LABELS,0);
		labeling(image,label,rows,cols,zeroAsBg);
std::cout << __FILE__ << __LINE__ << std::endl;		
		printf("labeling......\n");
		for(int i = 0;i < label.size();i++)
			std::cout << label[i] << " ";
		std::cout << std::endl;

		printf("next_label:%d\n",next_label);
		vector<int> stat_(next_label+1,0);

		for(int i = 0;i < image.size();i++)
			stat_[label[i]]++;

		stat_[0] = 0;
		int temp = 1;
		for(int i = 1;i < stat_.size();i++)
			if(stat_[i] != 0) stat_[i] = temp++;

		printf("stat_......\n");
		for(int i = 0;i < stat_.size();i++)
			std::cout << stat_[i] << " ";
		std::cout << std::endl;

		next_label = temp-1;
		int locIDX = 0;
		for(int i = 0;i < rows;i++)
			for(int j = 0;j < cols;j++)
			{
				label[locIDX] = stat_[label[locIDX]];
				label_2d_.at<int>(i,j) = label[locIDX];
				
				locIDX++;
			} 
	}

	void transform2Dto1D(cv::Mat& data_,vector<int>& vec_,int rows,int cols)
	{
		int k = 0;
		for(int i = 0;i < rows;i++)
			for(int j = 0;j < cols;j++)
			{
				vec_.push_back(data_.at<int>(i,j));
			}
	}

	void labeling(vector<int>& data_,vector<int>& vec_,int rows,int cols,bool zeroAsBg)
	{
		vector<int> parent(MAX_LABELS);
		vector<int> labels(MAX_LABELS);

		int next_region = 1;
		for(int y = 0;y < rows;y++){
			for(int x = 0;x < cols;x++){
				if(data_[y*cols+x] == 0 && zeroAsBg) continue;

				int k = 0;
				bool connected = false;

				if(x > 0 && data_[y*cols+x-1] == data_[y*cols+x])
				{
					k = vec_[y*cols+x-1];
					//printf("k1:%d\n",k);
					connected = true;
				}
//std::cout << __FILE__ << __LINE__ << std::endl;
				if(y > 0 && data_[(y-1)*cols+x] == data_[y*cols+x] && (!connected || data_[(y-1)*cols+x] < k))
				{
					k = vec_[(y-1)*cols+x];
					//printf("k2:%d\n",k);
					connected = true;
				}

				if(!connected)
				{
					k = next_region;
					//printf("k3:%d\n",k);
					next_region++;
				}

				vec_[y*cols+x] = k;

				if(x > 0 && data_[y*cols+x-1] == data_[y*cols+x] && vec_[y*cols+x-1] != k)
				{
					//std::cout << __FILE__ << __LINE__ << std::endl;
					uf_union(k,vec_[y*cols+x-1],parent);
					//std::cout << __FILE__ << __LINE__ << std::endl;
				}

				if(y > 0 && data_[(y-1)*cols+x] == data_[y*cols+x] && vec_[(y-1)*cols+x] != k)
				{
					uf_union(k,vec_[(y-1)*cols+x],parent);
				}

			}
		}
		next_label = 1;
		//std::cout << __FILE__ << __LINE__ << std::endl;
		for(int i = 0;i < rows*cols;i++){
			if(data_[i] != 0 || !zeroAsBg){
				vec_[i] = uf_find(vec_[i],parent,labels);
				if(!zeroAsBg) vec_[i]--;
			}
		}
		//std::cout << __FILE__ << __LINE__ << std::endl;
		next_label--;
		if(!zeroAsBg) next_label--;
	}

 	void uf_union( int x, int y, vector<int>& parent)
  {
			printf("start_x:%d\n",x);
			printf("start_y:%d\n",y);
//std::cout << __FILE__ << __LINE__ << std::endl;
      while ( parent[x]>0 ){
          x = parent[x];
					printf("x:%d\n",x);
			}
//std::cout << __FILE__ << __LINE__ << std::endl;
      while ( parent[y]>0 ){
          y = parent[y];
					printf("y:%d\n",y);
			}
//std::cout << __FILE__ << __LINE__ << std::endl;
      if ( x != y ) {
          if (x<y)
              parent[x] = y;
          else parent[y] = x;
      }
//std::cout << __FILE__ << __LINE__ << std::endl;
  }

  int uf_find( int x, vector<int>& parent, vector<int>& label)
  {
      while ( parent[x]>0 )
          x = parent[x];
      if ( label[x] == 0 )
          label[x] = next_label++;
      return label[x];
  }

};

class DarpRosNodelet : public nodelet::Nodelet {

	void getGraphics(cv::Mat& map_,cv::Mat& data_)
	{
		for(int i = 0;i < map_.rows;i++)
			for(int j = 0;j < map_.cols;j++)
			{
				if(map_.at<unsigned char>(i,j) == 128)
					data_.at<int>(i,j) = 1;
				else if(map_.at<unsigned char>(i,j) == 255)
					data_.at<int>(i,j) = 2;
				else
					data_.at<int>(i,j) = 0;
			}
	}

	void makeGridBinary(cv::Mat& data_,cv::Mat& binary_)
	{
		for(int i = 0;i < data_.rows;i++)
			for(int j = 0;j < data_.cols;j++)
			{
				if(data_.at<int>(i,j) != 1)
					binary_.at<int>(i,j) = 1;
				else
					binary_.at<int>(i,j) = 0;
			}
	}

	void call_back() {
		NODELET_INFO_STREAM("start Darp algorithm......\n");

		ConnectComponent cc(environment_grid_.rows,environment_grid_.cols);
		NODELET_INFO_STREAM("cc.rows:" << cc.rows);
		NODELET_INFO_STREAM("cc.cols:" << cc.cols);
		NODELET_INFO_STREAM("cc.MAX_LABELS:" << cc.MAX_LABELS);

		binary_grid_.create(cc.rows,cc.cols,CV_32SC1);
		makeGridBinary(environment_grid_,binary_grid_);

		NODELET_INFO_STREAM("binary_grid_:\n" << binary_grid_);

		label_2d_.create(cc.rows,cc.cols,CV_32SC1);
		cc.compactLabeling(binary_grid_,label_2d_,binary_grid_.rows,binary_grid_.cols,true);

		NODELET_INFO_STREAM("label_2d_:\n" << label_2d_);

		if(cc.getMaxLabel() > 1)
		{NODELET_INFO_STREAM("The environment grid MUST not have unreachable and/or closed shape regions\n\n");}

		DARP p(environment_grid_.rows,environment_grid_.cols,environment_grid_);
		
	}

	void onInit() {
		ros::NodeHandle pn("~");

		//pn.param<string>("map_path_string", map_path_, "/home/wzm/ine_detection_and_rotation_ros/src/line_detection_and_rotation/maps/test.bmp");	
		//NODELET_INFO_STREAM("map_path_:" << map_path_);

		std::cout << __FILE__ << __LINE__ << std::endl;
		
		int rows = 7;
		int cols = 7;

		unsigned char map_data[] = {
		0,0,0,0,0,0,0,
		0,0,0,0,0,0,0,
		0,0,0,128,0,0,0,
		0,0,0,128,0,255,0,
		0,0,0,128,0,0,0,
		0,0,0,0,0,0,0,
		0,0,0,0,0,0,0};

		cv::Mat map_(rows,cols,CV_8UC1,map_data);

		if(map_.empty())
		{
			std::cout << "map open fail.............." << std::endl;
			return;
		}
		else{
			NODELET_INFO_STREAM("map_width:" << map_.cols);
			NODELET_INFO_STREAM("map_height:" << map_.rows);
			
			environment_grid_.create(map_.rows,map_.cols,CV_32SC1);
			getGraphics(map_,environment_grid_);

			NODELET_INFO_STREAM("environment_grid_:\n" << environment_grid_);

			darp_thread = boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&DarpRosNodelet::call_back, this)));
		}
	}

	~DarpRosNodelet() {		
	}

};

}
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(darp_ros::DarpRosNodelet, nodelet::Nodelet);

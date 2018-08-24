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
#include "line_detection_and_rotation/MapRotate.h"

namespace line_detection_and_rotation {

	using namespace std;
	using namespace cv;

	typedef struct
	{
			int nValue;
			int nDist;
			int nAngleNum;	
	}LineData;

	void ImgRotate(cv::Mat& src_,cv::Mat& dst_,bool direction,int angle)
	{
		
		int oldWidth = src_.cols;
		int oldHeight = src_.rows;

		float fSrcX1, fSrcY1, fSrcX2, fSrcY2, fSrcX3, fSrcY3, fSrcX4, fSrcY4;
		fSrcX1 = (float)(-(oldWidth) / 2.0);
		fSrcY1 = (float)((oldHeight) / 2.0);
		fSrcX2 = (float)((oldWidth ) / 2.0);
		fSrcY2 = (float)((oldHeight) / 2.0);
		fSrcX3 = (float)(-(oldWidth) / 2.0);
		fSrcY3 = (float)(-(oldHeight) / 2.0);
		fSrcX4 = (float)((oldWidth) / 2.0);
		fSrcY4 = (float)(-(oldHeight) / 2.0);	

		//NODELET_INFO_STREAM("fSrcX1:" << fSrcX1);
		//NODELET_INFO_STREAM("fSrcY1:" << fSrcY1);
		//NODELET_INFO_STREAM("fSrcX2:" << fSrcX2);
		//NODELET_INFO_STREAM("fSrcY2:" << fSrcY2);
		//NODELET_INFO_STREAM("fSrcX3:" << fSrcX3);
		//NODELET_INFO_STREAM("fSrcY3:" << fSrcY3);
		//NODELET_INFO_STREAM("fSrcX4:" << fSrcX4);
		//NODELET_INFO_STREAM("fSrcY4:" << fSrcY4);
		float theta = angle*CV_PI*direction/180.0;

		//NODELET_INFO_STREAM("theta:" << theta);
		float fDstX1, fDstY1, fDstX2, fDstY2, fDstX3, fDstY3, fDstX4, fDstY4;

		fDstX1 = cos(theta) * fSrcX1 + sin(theta) * fSrcY1;
		fDstY1 = -sin(theta) * fSrcX1 + cos(theta) * fSrcY1;
		fDstX2 = cos(theta) * fSrcX2 + sin(theta) * fSrcY2;
		fDstY2 = -sin(theta) * fSrcX2 + cos(theta) * fSrcY2;
		fDstX3 = cos(theta) * fSrcX3 + sin(theta) * fSrcY3;
		fDstY3 = -sin(theta) * fSrcX3 + cos(theta) * fSrcY3;
		fDstX4 = cos(theta) * fSrcX4 + sin(theta) * fSrcY4;
		fDstY4 = -sin(theta) * fSrcX4 + cos(theta) * fSrcY4;
		
		//NODELET_INFO_STREAM("fDstX1:" << fDstX1);
		//NODELET_INFO_STREAM("fDstY1:" << fDstY1);
		//NODELET_INFO_STREAM("fDstX2:" << fDstX2);
		//NODELET_INFO_STREAM("fDstY2:" << fDstY2);
		//NODELET_INFO_STREAM("fDstX3:" << fDstX3);
		//NODELET_INFO_STREAM("fDstY3:" << fDstY3);
		//NODELET_INFO_STREAM("fDstX4:" << fDstX4);
		//NODELET_INFO_STREAM("fDstY4:" << fDstY4);
		int newWidth =  (int)((fabs(fDstX4 - fDstX1) > fabs(fDstX3 - fDstX2)? fabs(fDstX4 - fDstX1): fabs(fDstX3 - fDstX2))+0.5);
		int newHeight = (int)((fabs(fDstY4 - fDstY1) > fabs(fDstY3 - fDstY2)? fabs(fDstY4 - fDstY1): fabs(fDstY3 - fDstY2))+0.5);

		//NODELET_INFO_STREAM("newWidth:" << newWidth);
		//NODELET_INFO_STREAM("newHeight:" << newHeight);
		std::cout << __FILE__ << __LINE__ << std::endl;
		cv::Mat dst(newHeight,newWidth,CV_8UC3);

		std::cout << __FILE__ << __LINE__ << std::endl;
		float dx = -0.5*newWidth*cos(theta) - 0.5*newHeight*sin(theta) + 0.5*oldWidth;
		float dy = 0.5*newWidth*sin(theta) - 0.5*newHeight*cos(theta) + 0.5*oldHeight;
		
		//NODELET_INFO_STREAM("channels:" << src_.channels());
		//NODELET_INFO_STREAM("dx:" << dx);
		//NODELET_INFO_STREAM("dy:" << dy);
		for (int h = 0; h < newHeight; h++)
		{
			for (int w = 0; w < newWidth; w++)
			{
				int x = float(w)*cos(theta) + float(h)*sin(theta) + dx;
				int y = float(-w)*sin(theta) + float(h)*cos(theta) + dy;
				
				//NODELET_INFO_STREAM("x:" << x);
				//NODELET_INFO_STREAM("y:" << y);
				if ((x < 0) || (x >= oldWidth) || (y < 0) || (y >= oldHeight))
				{
					if (src_.channels() == 3)
					{
						dst.at<cv::Vec3b >(h,w)[0] = 0;
						dst.at<cv::Vec3b >(h,w)[1] = 0;
						dst.at<cv::Vec3b >(h,w)[2] = 0;
					}
					else if (src_.channels() == 1)
					{
						dst.at<unsigned char>(h,w) = 0;
					}
				}
				else
				{
					if (src_.channels() == 3)
					{
						dst.at<cv::Vec3b >(h,w)[0] = src_.at<cv::Vec3b >(y,x)[0];
						dst.at<cv::Vec3b >(h,w)[1] = src_.at<cv::Vec3b >(y,x)[1];
						dst.at<cv::Vec3b >(h,w)[2] = src_.at<cv::Vec3b >(y,x)[2];
					}
					else if (src_.channels() == 1)
					{
						dst.at<unsigned char>(h,w) = src_.at<unsigned char>(y,x);
					}
				}
			}
		}
		dst_ = dst;
	}

	bool lineHoughTransform(vector<cv::Point>& src,int& dist,int& value,int& angle,int width,int height,int count)
	{
		int nRow = height;
		int nCol = width;
		int nCount = count;

		int nDist;
		int nAngleNumber;

		int nMaxAngleNumber = 180;
		int nMaxDist =(int)sqrt(nRow * nRow + nCol * nCol);
		int* pTran = new int[nMaxDist * nMaxAngleNumber];
		memset(pTran,0,nMaxDist * nMaxAngleNumber * sizeof(int));

		int i,j;
		int ni;


		std::cout << __FILE__ << __LINE__ << std::endl;
		for(ni = 0;ni < nCount;ni++)
		{
		 	for(nAngleNumber = 0;nAngleNumber < nMaxAngleNumber;nAngleNumber++)
		  {
		    nDist = round(src[ni].x*cos(nAngleNumber*CV_PI/180)+src[ni].y*sin(nAngleNumber*CV_PI/180)+0.05);
				if(nDist > 0)
		      pTran[nDist*nMaxAngleNumber+nAngleNumber] = pTran[nDist*nMaxAngleNumber+nAngleNumber]+1;
		  }
		}


		std::cout << __FILE__ << __LINE__ << std::endl;
		LineData line;
		line.nValue = 0; 
		line.nDist = 0;
		line.nAngleNum = 0;
		 
		bool reFlags = false;
		int nScale =3;

		for(nDist = nRow;nDist > 0;nDist--)
		{
		  for(nAngleNumber = 0;nAngleNumber < 180;nAngleNumber++)
		  {
		    if(pTran[nDist * nMaxAngleNumber + nAngleNumber] > line.nValue)
		     {
		       line.nValue = pTran[nDist * nMaxAngleNumber + nAngleNumber];
		       line.nDist = nDist;
		       line.nAngleNum = nAngleNumber;
					 reFlags = true;
		     }
		   }
		}

		dist = line.nDist;
		angle = line.nAngleNum;
		value = line.nValue;

		delete[] pTran;

		if(reFlags)
			return true;
		else
			return false;
	}

	bool MapRotate(line_detection_and_rotation::MapRotate::Request &req,     
    			   line_detection_and_rotation::MapRotate::Response &res) {
		ROS_INFO("start rotate map...");
		return true;
#if 0
		ros::NodeHandle pn("~");

		//pn.param<string>("map_path_string", map_path_, "/home/wzm/ine_detection_and_rotation_ros/src/line_detection_and_rotation/maps/test.bmp");
		//pn.param<int>("binary_threshold",threshold_,95);

		//std::cout << __FILE__ << __LINE__ << std::endl;
		//src_ = 	imread(map_path_);
		
		//std::cout << __FILE__ << __LINE__ << std::endl;
		if(src_.empty())
		{
			std::cout << "map_image open fail.............." << std::endl;
			return;
		}
		else{
					//NODELET_INFO_STREAM("map_width:" << src_.cols);
					//NODELET_INFO_STREAM("map_height:" << src_.rows);
					cv::Mat binarization;
					cv::Canny(src_,binarization,75,3,3);


					vector<cv::Point> map_edge;

					for(int i = 0;i < binarization.cols;i++)
						for(int j = 0;j < binarization.rows;j++)
						{
							if(binarization.at<unsigned char>(j,i) == 255)
								map_edge.push_back(cv::Point(i,j));
						}
					int nValue = -1;
					int nDist = -1;
					int nAngle = -1;

					NODELET_INFO_STREAM("map_edge_size:" << map_edge.size());
					std::cout << __FILE__ << __LINE__ << std::endl;

					bool res = lineHoughTransform(map_edge,nDist,nValue,nAngle,binarization.cols,binarization.rows,map_edge.size());

					std::cout << __FILE__ << __LINE__ << std::endl;
					if(!res)	
					{
						std::cout << "detetion line fail.................." << std::endl;
						return;
					}
					else		
						ImgRotate(src_,dst_,1,nAngle+15);	
		}
#endif
	}

}
int main(int argc, char **argv) {
  ros::init(argc, argv, "darp_path_plan_node");

  ros::NodeHandle private_nh("~");

  ros::ServiceServer map_rotate_srv = private_nh.advertiseService(
      "/sweeper/map_rotate_srv",
      line_detection_and_rotation::MapRotate);

  ROS_INFO("Ready to Map rotate...");

  ros::spin();

  return 0;
}

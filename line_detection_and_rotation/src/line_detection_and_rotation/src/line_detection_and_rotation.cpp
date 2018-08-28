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

	bool ImgRotate(cv::Mat& src_,cv::Mat& dst_,bool direction,int angle)
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

		float theta = angle*CV_PI*direction/180.0;

		float fDstX1, fDstY1, fDstX2, fDstY2, fDstX3, fDstY3, fDstX4, fDstY4;

		fDstX1 = cos(theta) * fSrcX1 + sin(theta) * fSrcY1;
		fDstY1 = -sin(theta) * fSrcX1 + cos(theta) * fSrcY1;
		fDstX2 = cos(theta) * fSrcX2 + sin(theta) * fSrcY2;
		fDstY2 = -sin(theta) * fSrcX2 + cos(theta) * fSrcY2;
		fDstX3 = cos(theta) * fSrcX3 + sin(theta) * fSrcY3;
		fDstY3 = -sin(theta) * fSrcX3 + cos(theta) * fSrcY3;
		fDstX4 = cos(theta) * fSrcX4 + sin(theta) * fSrcY4;
		fDstY4 = -sin(theta) * fSrcX4 + cos(theta) * fSrcY4;
		
		int newWidth =  (int)((fabs(fDstX4 - fDstX1) > fabs(fDstX3 - fDstX2)? fabs(fDstX4 - fDstX1): fabs(fDstX3 - fDstX2))+0.5);
		int newHeight = (int)((fabs(fDstY4 - fDstY1) > fabs(fDstY3 - fDstY2)? fabs(fDstY4 - fDstY1): fabs(fDstY3 - fDstY2))+0.5);

		//std::cout << __FILE__ << __LINE__ << std::endl;
		cv::Mat dst(newHeight,newWidth,CV_8UC1);

		//std::cout << __FILE__ << __LINE__ << std::endl;
		float dx = -0.5*newWidth*cos(theta) - 0.5*newHeight*sin(theta) + 0.5*oldWidth;
		float dy = 0.5*newWidth*sin(theta) - 0.5*newHeight*cos(theta) + 0.5*oldHeight;
		
		for (int h = 0; h < newHeight; h++)
		{
			for (int w = 0; w < newWidth; w++)
			{
				int x = float(w)*cos(theta) + float(h)*sin(theta) + dx;
				int y = float(-w)*sin(theta) + float(h)*cos(theta) + dy;
				
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
						dst.at<unsigned char>(h,w) = 100;
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
		int nMaxDist =(int)sqrt(nRow/2.0 * nRow/2.0 + nCol/2.0 * nCol/2.0);
		int* pTran = new int[nMaxDist * nMaxAngleNumber * 2];
		memset(pTran,0,nMaxDist * nMaxAngleNumber * sizeof(int));

		int i,j;
		int ni;


		//std::cout << __FILE__ << __LINE__ << std::endl;
		for(ni = 0;ni < nCount;ni++)
		{
		 	for(nAngleNumber = -nMaxAngleNumber;nAngleNumber < nMaxAngleNumber;nAngleNumber++)
		    {   
                int x = src[ni].x-nCol/2;
                int y = nRow-src[ni].y-nRow/2;

		        nDist = round(x*cos(nAngleNumber*CV_PI/180)+y*sin(nAngleNumber*CV_PI/180)+0.05);
				if(nDist > 0)
		            pTran[nDist*nMaxAngleNumber+nAngleNumber] = pTran[nDist*nMaxAngleNumber+nAngleNumber]+1;
		    }
		}


		//std::cout << __FILE__ << __LINE__ << std::endl;
		LineData line;
		line.nValue = 0; 
		line.nDist = 0;
		line.nAngleNum = 0;
		 
		bool reFlags = false;
		for(nDist = nMaxDist;nDist > 0;nDist--)
		{
		  for(nAngleNumber = -nMaxAngleNumber;nAngleNumber < nMaxAngleNumber;nAngleNumber++)
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
		
		if(req.map.info.width*req.map.info.height != req.map.data.size()){
				ROS_ERROR("map data error...");
				return false;
		}

        for(int i = 0;i < req.map.data.size();i++)
        if(req.map.data[i] == -1)
            req.map.data[i] = 100;
   
        cv::Mat map(req.map.info.height, req.map.info.width, CV_8UC1, req.map.data.data());

		if(!map.empty()){
				cv::Mat binarization;
				cv::Canny(map,binarization,75,3,3);

				vector<cv::Point> map_edge;
				for(int i = 0;i < binarization.cols;i++)
						for(int j = 0;j < binarization.rows;j++)
						{
							if(binarization.at<unsigned char>(j,i) == 255)
								map_edge.push_back(cv::Point(i,j));
						}
				int nValue = -1;
				int nDist = -1;
				int nAngle = -111111;

				//std::cout << __FILE__ << __LINE__ << std::endl;

				bool res_line = lineHoughTransform(map_edge,nDist,nValue,nAngle,binarization.cols,binarization.rows,map_edge.size());

				//std::cout << __FILE__ << __LINE__ << std::endl;
				if(!res_line)	
				{
                        ROS_ERROR("detetion line fail...");
						return false;
				}
				else{
                        ROS_INFO("line detection suc;cess...");
                        ROS_INFO("nValue:%d",nValue);
                        ROS_INFO("nDist:%d",nDist);
                        ROS_INFO("nAngle:%d",nAngle);
						cv::Mat dst;		
						ImgRotate(map,dst,1,nAngle);
						if(!dst.empty()){
                            ROS_INFO("dst width:%d",dst.cols);
                            ROS_INFO("dst height:%d",dst.rows);

                            vector<int8_t> map_data;
				            for(int i = 0;i < dst.rows;i++){
				                for(int j = 0;j < dst.cols;j++){
				                    char value = dst.at<char>(i,j);
				                    map_data.push_back(value); 
				                }
				            }

				            res.map.info.height = dst.rows;
				            res.map.info.width = dst.cols;
				            res.map.info.resolution = req.map.info.resolution;
				            res.map.data = map_data;

				            res.map.header.frame_id = req.map.header.frame_id;
				            res.map.info.origin.position.x = req.map.info.origin.position.x;
				            res.map.info.origin.position.y = req.map.info.origin.position.y;
						    
                            return true;
						}
						else
							return false;
				}
		}
		else{
				ROS_ERROR("map data empty...");
				return false;
		}
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

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

namespace line_detection_and_rotation {

using namespace std;
using namespace cv;

typedef struct
{
	int nValue;
	int nDist;
	int nAngleNum;	
}LineData;

class LineDetectionAndRotationNodelet : public nodelet::Nodelet {

	cv::Mat src_;
	string map_path_;
	int threshold_;

	boost::shared_ptr<boost::thread> device_poll_thread;


	void ImgRotate(cv::Mat& src_,cv::Mat& dst_,bool direction,int angle)
	{
		int oldWidth = src_.cols;
		int oldHeight = src_.rows;

		float fSrcX1, fSrcY1, fSrcX2, fSrcY2, fSrcX3, fSrcY3, fSrcX4, fSrcY4;
		fSrcX1 = (float)(-(oldWidth) / 2);
		fSrcY1 = (float)((oldHeight) / 2);
		fSrcX2 = (float)((oldWidth ) / 2);
		fSrcY2 = (float)((oldHeight) / 2);
		fSrcX3 = (float)(-(oldWidth) / 2);
		fSrcY3 = (float)(-(oldHeight) / 2);
		fSrcX4 = (float)((oldWidth) / 2);
		fSrcY4 = (float)(-(oldHeight) / 2);	

		float theta = 0.5*CV_PI*direction;
		float fDstX1, fDstY1, fDstX2, fDstY2, fDstX3, fDstY3, fDstX4, fDstY4;

		fDstX1 = cos(theta) * fSrcX1 + sin(theta) * fSrcY1;
		fDstY1 = -sin(theta) * fSrcX1 + cos(theta) * fSrcY1;
		fDstX2 = cos(theta) * fSrcX2 + sin(theta) * fSrcY2;
		fDstY2 = -sin(theta) * fSrcX2 + cos(theta) * fSrcY2;
		fDstX3 = cos(theta) * fSrcX3 + sin(theta) * fSrcY3;
		fDstY3 = -sin(theta) * fSrcX3 + cos(theta) * fSrcY3;
		fDstX4 = cos(theta) * fSrcX4 + sin(theta) * fSrcY4;
		fDstY4 = -sin(theta) * fSrcX4 + cos(theta) * fSrcY4;
		
		int newWidth =  (max(fabs(fDstX4 - fDstX1), fabs(fDstX3 - fDstX2)) );
		int newHeight = (max(fabs(fDstY4 - fDstY1), fabs(fDstY3 - fDstY2)) );

		cv::Mat dst(newHeight,newWidth,CV_8UC1);

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
						dst.at<cv::Vec3b >(h,w)[0] = 0;
						dst.at<cv::Vec3b >(h,w)[0] = 0;
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
						dst.at<cv::Vec3b >(h,w)[0] = src_.at<cv::Vec3b >(y,x)[1];
						dst.at<cv::Vec3b >(h,w)[0] = src_.at<cv::Vec3b >(y,x)[2];
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

	bool lineHoughTransform(vector<cv::Point>& src,int dist,int value,int angle,int width,int height,int count)
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


		for(ni = 0;ni < nCount;ni++)
		{
		 	for(nAngleNumber = 0;nAngleNumber < nMaxAngleNumber;nAngleNumber++)
		  {
		    nDist = round(src[ni].x*cos(nAngleNumber*CV_PI/180)+src[ni].y*sin(nAngleNumber*CV_PI/180)+0.05);
				if(nDist > 0)
		      pTran[nDist*nMaxAngleNumber+nAngleNumber] = pTran[nDist*nMaxAngleNumber+nAngleNumber]+1;
		  }
		}


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

	void device_poll() {

		cv::Mat binarization;
		cv::threshold(
		    src_, binarization, threshold_, 255, cv::THRESH_BINARY_INV);

		cv::Mat erosion, element;
		element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
		cv::erode(
		    binarization, erosion, element, cv::Point(-1, -1),
		    3);

		vector <vector<Point> > map_contours;  
  	findContours( erosion,map_contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);

		int nValue = -1;
		int nDist = -1;
		int nAngle = -1;

		lineHoughTransform(map_contours[0],nDist,nValue,nAngle,erosion.cols,erosion.rows,map_contours[0].size());

		if(nDist < 0 || nValue < 0 || nAngle)	
		{
			std::cout << "detetion line fail.................." << std::endl;
			return;
		}
	}

	void onInit() {
		ros::NodeHandle pn("~");

		pn.param<string>("map_path_string", map_path_, "../maps/a.pgm");
		pn.param<int>("binary_threshold",threshold_,75);
		
		src_ = 	imread(map_path_);

		if(src_.empty())
		{
			std::cout << "map_image open fail.............." << std::endl;
			return;
		}
		else{
				device_poll_thread = boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&LineDetectionAndRotationNodelet::device_poll, this)));
		}
	}

	~LineDetectionAndRotationNodelet() {		
		src_.release();
	}

};

}
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(line_detection_and_rotation::LineDetectionAndRotationNodelet, nodelet::Nodelet);

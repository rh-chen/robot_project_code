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
#include "scale_map/MapRotate.h"
#include "lsd.h"

namespace line_detection_and_rotation {

	using namespace std;
	using namespace cv;

	typedef struct
	{
			int nValue;
			int nDist;
			int nAngleNum;	
	}LineData;

	bool ImgRotate(cv::Mat& src_,cv::Mat& dst_,int direction,double angle,vector<double>& rot_mat)
	{
#if 0
        double scale = 1.0;
        cv::Point2f image_center(src_.cols/2.0,src_.rows/2.0);
    
        cv::Mat rotate_mat = cv::getRotationMatrix2D(image_center,direction*angle,scale);

        std::cout << "rotate_mat:" << rotate_mat << std::endl;

        cv::Rect box;
        box = cv::RotatedRect(image_center,cv::Size(scale*src_.cols,scale*src_.rows),angle).boundingRect();

        rotate_mat.at<double>(0,2) += box.width/2 - image_center.x;
        rotate_mat.at<double>(1,2) += box.height/2 - image_center.y;

        cv::Mat dst;
        cv::warpAffine(src_,dst,rotate_mat, box.size());

        cv::Mat rotate_mat_3(3,3,CV_64FC1);
        rotate_mat_3.at<double>(0,0) = rotate_mat.at<double>(0,0);
        rotate_mat_3.at<double>(0,1) = rotate_mat.at<double>(0,1);
        rotate_mat_3.at<double>(0,2) = rotate_mat.at<double>(0,2);
        rotate_mat_3.at<double>(1,0) = rotate_mat.at<double>(1,0);
        rotate_mat_3.at<double>(1,1) = rotate_mat.at<double>(1,1);
        rotate_mat_3.at<double>(1,2) = rotate_mat.at<double>(1,2);
        rotate_mat_3.at<double>(2,0) = 0;
        rotate_mat_3.at<double>(2,1) = 0;
        rotate_mat_3.at<double>(2,2) = 1;

        cv::Mat rotate_mat_inv;
        rotate_mat_inv = rotate_mat_3.inv();

        std::cout << "rotate_mat_inv:" << rotate_mat_inv << std::endl;

        for(int i = 0;i < rotate_mat_inv.rows;i++){
            for(int j = 0;j < rotate_mat_inv.cols;j++){
                rot_mat.push_back(rotate_mat_inv.at<double>(i,j));
            }
        }
#endif
#if 1
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
    
        rot_mat.push_back(cos(theta));
        rot_mat.push_back(sin(theta));
        rot_mat.push_back(dx);
        rot_mat.push_back(-sin(theta));
        rot_mat.push_back(cos(theta));
        rot_mat.push_back(dy);
        rot_mat.push_back(0);
        rot_mat.push_back(0);
        rot_mat.push_back(1);

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
#endif
        dst_ = dst;
	}

	bool lineLsdTransform(cv::Mat& bin_,double& angle)
	{
		cv::Mat bin_temp;
        bin_.convertTo(bin_temp,CV_64FC1);

        //std::cout << __FILE__ << __LINE__ << std::endl;
        image_double image = new_image_double(bin_temp.cols,bin_temp.rows);
        image->data = bin_temp.ptr<double>(0);

        //std::cout << __FILE__ << __LINE__ << std::endl;
        ntuple_list ntl = lsd(image);

        //std::cout << __FILE__ << __LINE__ << std::endl;
        ROS_INFO("Lines Number:%d",ntl->size);
        double max_length = 0.0;
        double line_angle = 0.0;
        int line_num = ntl->size;

        for(int j = 0;j < line_num;j++){
            cv::Point start,end;

            start.x = int(ntl->values[0 + j * ntl->dim]);
            start.y = int(ntl->values[1 + j * ntl->dim]);
            end.x = int(ntl->values[2 + j * ntl->dim]);
            end.y = int(ntl->values[3 + j * ntl->dim]);
        
            int delta_x = end.x-start.x;
            int delta_y = end.y-start.y;

            if(sqrt(delta_x*delta_x+delta_y*delta_y) > max_length){
                //line_angle = atan2(delta_y,delta_x);
                line_angle = cv::fastAtan2(delta_y,delta_x);
                max_length = sqrt(delta_x*delta_x+delta_y*delta_y);
            }
	    }

        //angle = (line_angle*180.0)/CV_PI+180.0;
        angle = line_angle;
        ROS_INFO("line_anlge:%f",angle);
        ROS_INFO("max_length:%f",max_length);
    
        free_ntuple_list(ntl);

        if(line_num > 0)
            return true;
        else
            return false;
#if 0
		int nRow = height;
		int nCol = width;
		int nCount = count;

		int nDist;
		int nAngleNumber;

		int nMaxAngleNumber = 360;
		int nMaxDist =(int)sqrt(nRow/2.0 * nRow/2.0 + nCol/2.0 * nCol/2.0);
		int* pTran = new int[nMaxDist * nMaxAngleNumber];
		memset(pTran,0,nMaxDist * nMaxAngleNumber * sizeof(int));

		int i,j;
		int ni;


		//std::cout << __FILE__ << __LINE__ << std::endl;
		for(ni = 0;ni < nCount;ni++)
		{
		 	for(nAngleNumber = 0;nAngleNumber < nMaxAngleNumber;nAngleNumber++)
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
		  for(nAngleNumber = 0;nAngleNumber < nMaxAngleNumber;nAngleNumber++)
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
#endif
	}

	bool MapRotate(scale_map::MapRotate::Request &req,     
    			   scale_map::MapRotate::Response &res) {
		ROS_INFO("start rotate map...");
		
		if(req.map.info.width*req.map.info.height != req.map.data.size()){
				ROS_ERROR("map data error...");
				return false;
		}
        
        ROS_INFO("map.info.origin.position.x:%f",req.map.info.origin.position.x);
        ROS_INFO("map.info.origin.position.y:%f",req.map.info.origin.position.y);
        ROS_INFO("map.info.resolution:%f",req.map.info.resolution);

        for(int i = 0;i < req.map.data.size();i++)
        if(req.map.data[i] == -1)
            req.map.data[i] = 100;
   
        cv::Mat map(req.map.info.height, req.map.info.width, CV_8UC1, req.map.data.data());

		if(!map.empty()){
				cv::Mat bin;
				cv::threshold(map,bin,95,255,cv::THRESH_BINARY_INV);
				/*cv::Canny(map,binarization,75,3,3);
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
				*/
				//std::cout << __FILE__ << __LINE__ << std::endl;
                double nAngle = 0;
                ros::Time begin_line = ros::Time::now();
				bool res_line = lineLsdTransform(bin,nAngle);
                ros::Time end_line = ros::Time::now();
                std::cout << "line_hough_time_cost:" << (end_line-begin_line).toSec() << std::endl;
				//std::cout << __FILE__ << __LINE__ << std::endl;
				if(!res_line)	
				{
                        ROS_ERROR("detetion line fail...");
						return false;
				}
				else{
                        ROS_INFO("line detection success...");
                        ROS_INFO("nAngle:%f",nAngle);
						cv::Mat dst;		
						double angle_threshold = 3;
                        double angle_rotate = 0;
                        if(floor(nAngle) >= 0 && floor(nAngle) < 90)
                            angle_rotate = 90-nAngle;
                        else if(floor(nAngle) >= 90 && floor(nAngle) < 180)
                            angle_rotate = 180-nAngle;
                        else if(floor(nAngle) >= 180 && floor(nAngle) < 270)
                            angle_rotate = 270-nAngle;
                        else if(floor(nAngle) >= 270 && floor(nAngle) < 360)
                            angle_rotate = 360-nAngle;
                       
                        std::vector<double> rot_mat_inv;
                        ROS_INFO("angle_rotate:%f",angle_rotate);
                        if(angle_rotate > angle_threshold){
                            if((90-angle_rotate) < angle_rotate){
                                ros::Time begin_rotate = ros::Time::now();
						        ImgRotate(map,dst,-1,90-angle_rotate,rot_mat_inv);
                                ros::Time end_rotate = ros::Time::now();
                                std::cout << "image_rotate_time_cost:" << (end_rotate-begin_rotate).toSec() << std::endl;
                                std::cout << "rot_mat_inv:" << rot_mat_inv[0] << "  "
                                                            << rot_mat_inv[1] << "  "
                                                            << rot_mat_inv[2] << "  " << std::endl;

                                std::cout << rot_mat_inv[3] << "  "
                                          << rot_mat_inv[4] << "  "
                                          << rot_mat_inv[5] << "  " << std::endl;

                                std::cout << rot_mat_inv[6] << "  "
                                          << rot_mat_inv[7] << "  "
                                          << rot_mat_inv[8] << "  " << std::endl;
                            }
                            else{    
                                ros::Time begin_rotate = ros::Time::now();
						        ImgRotate(map,dst,1,angle_rotate,rot_mat_inv);
                                ros::Time end_rotate = ros::Time::now();
                                std::cout << "image_rotate_time_cost:" << (end_rotate-begin_rotate).toSec() << std::endl;
                                std::cout << "rot_mat_inv:" << rot_mat_inv[0] << "  "
                                                            << rot_mat_inv[1] << "  "
                                                            << rot_mat_inv[2] << "  " << std::endl;

                                std::cout << rot_mat_inv[3] << "  "
                                          << rot_mat_inv[4] << "  "
                                          << rot_mat_inv[5] << "  " << std::endl;

                                std::cout << rot_mat_inv[6] << "  "
                                          << rot_mat_inv[7] << "  "
                                          << rot_mat_inv[8] << "  " << std::endl;
                            }
                        }
                        else{
                            ros::Time begin_rotate = ros::Time::now();
                            ImgRotate(map,dst,1,0,rot_mat_inv);
                            ros::Time end_rotate = ros::Time::now();
                            std::cout << "image_rotate_time_cost:" << (end_rotate-begin_rotate).toSec() << std::endl;
                        }

						if(!dst.empty()){
                            ROS_INFO("dst width:%d",dst.cols);
                            ROS_INFO("dst height:%d",dst.rows);

                            ROS_INFO("map width:%d",map.cols);
                            ROS_INFO("map height:%d",map.rows);
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
                            
                            res.transform = rot_mat_inv;
                            res.map_origin_x = req.map.info.origin.position.x;
                            res.map_origin_y = req.map.info.origin.position.y;
                            res.map_resolution = req.map.info.resolution;

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
  ros::init(argc, argv, "map_rotate_node");

  ros::NodeHandle private_nh("~");

  ros::ServiceServer map_rotate_srv = private_nh.advertiseService(
      "/sweeper/map_rotate_srv",
      line_detection_and_rotation::MapRotate);

  ROS_INFO("Ready to Map rotate...");

  ros::spin();

  return 0;
}

/*ros*/
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

/*camera*/
#include "stdio.h"
#include "stdlib.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "ImiNect.h"


#define DEPTH_IMAGE_WIDTH			640
#define DEPTH_IMAGE_HEIGHT			480

#define COLOR_IMAGE_WIDTH			640
#define COLOR_IMAGE_HEIGHT			480

#define DEFAULT_PERPIXEL_BITS	24
#define DEFAULT_FRAMERATE		30

#define IMI_AR


//stream handles
uint32_t		g_streamNum = 0;
ImiStreamHandle g_streams[10] = { NULL };

ImiDeviceAttribute* pDeviceAttr = NULL;
ImiDeviceHandle pImiDevice = NULL;

//int32_t avStreamIndex = 0;
bool g_bColorFrameOK = false;
bool g_bDepthFrameOK = false;

bool init_camera()
{
#ifdef IMI_AR
	//1.imiInitialize()
	uint32_t deviceCount = 0;
	if (0 != imiInitialize(NULL)) 
	{
		printf("ImiNect Init Failed!\n");
		getchar();
		return false;
	}
	printf("ImiNect Init Success.\n");

	//2.imiGetDeviceList()
	imiGetDeviceList(&pDeviceAttr, &deviceCount);
	if( deviceCount <= 0 || NULL == pDeviceAttr ) 
	{
		printf("Get No Connected Imidevice!");
		imiDestroy();
		getchar();
		return false;
	}
	printf("Get %d Connected Imidevice.\n", deviceCount);

	//3.imiOpenDevice()
	if( 0 != imiOpenDevice(pDeviceAttr[0].uri, &pImiDevice, 0) )
	{
		printf("Open Imidevice Failed!\n");
		//release_ar();
		return false;
	}
	printf("Imidevice Opened.\n");

	if(pDeviceAttr[0].vendorId == 0x2dbb && pDeviceAttr[0].productId == 0x0002)//A100M device
	{
		//A100M device open image registration
		int registrationMode;
		registrationMode = 1;
		imiSetDeviceProperty( pImiDevice, IMI_PROPERTY_IMAGE_REGISTRATION, &registrationMode, sizeof(registrationMode));
		printf("Open Registration Mode Success!!!\n");
	}

	//4.imiOpenStream()
	ImiFrameMode frameMode;
	frameMode.pixelFormat = IMI_PIXEL_FORMAT_IMAGE_RGB24;
	frameMode.resolutionX = COLOR_IMAGE_WIDTH;
	frameMode.resolutionY = COLOR_IMAGE_HEIGHT;
	frameMode.bitsPerPixel = DEFAULT_PERPIXEL_BITS;
	frameMode.framerate = DEFAULT_FRAMERATE;
	imiSetFrameMode(pImiDevice, IMI_COLOR_FRAME, &frameMode);

	if (0 != imiOpenStream(pImiDevice, IMI_COLOR_FRAME, NULL, NULL, &g_streams[g_streamNum++])) {
		printf("Open Color Stream Failed!\n");
		//release_ar();
		return false;
	}
	printf("Open Color Stream Success.\n");


	//ImiFrameMode frameMode;

	frameMode.pixelFormat = IMI_PIXEL_FORMAT_DEP_16BIT;
	frameMode.resolutionX = DEPTH_IMAGE_WIDTH;
	frameMode.resolutionY = DEPTH_IMAGE_HEIGHT;
	frameMode.bitsPerPixel = DEFAULT_PERPIXEL_BITS;
	frameMode.framerate = DEFAULT_FRAMERATE;
	imiSetFrameMode(pImiDevice, IMI_DEPTH_FRAME, &frameMode);

	if ( 0 != imiOpenStream(pImiDevice, IMI_DEPTH_FRAME, NULL, NULL, &g_streams[g_streamNum++]) )
	{
		printf("Open Skeleton Stream Failed!\n");
		//release_ar();
		return false;
	}
	printf("Open Skeleton Stream Success.\n");
	
	return true;
#endif
}

//结束释放资源
bool release_camera()
{
#if (defined IMI_AR) || (defined IMI_AR_UVC)
	for (uint32_t num = 0; num < g_streamNum; ++num)
	{
        if (NULL != g_streams[num])
		{
            imiCloseStream(g_streams[num]);
        }
    }

	//7.imiCloseDevice()
    if (NULL != pImiDevice)
	{
        imiCloseDevice(pImiDevice);
    }

	//8.imiReleaseDeviceList()
    if(NULL != pDeviceAttr)
	{
        imiReleaseDeviceList(&pDeviceAttr);
    }

	//9.imiDestroy()
    imiDestroy();
#endif

	return true;
}

//获取彩色和深度图像
bool get_img_data(cv::Mat imRGB,cv::Mat imD)
{
/*#ifdef IMI_AR
	static uint64_t depth_t;
	static uint64_t color_t;

	ImiImageFrame* pFrameColor = NULL;
	ImiImageFrame* pFrameDepth = NULL;

	if (g_bColorFrameOK != true)
	{
		printf("g_streamNum:%d\n",g_streamNum);
		printf("avStreamIndex:%d\n",avStreamIndex);

		if(0 != imiWaitForStreams(g_streams, g_streamNum, &avStreamIndex, -1))
		{
			//usleep(500*1000);
			//return false;
			printf("wait stream fail..............\n");
		}
		
		printf("wait stream success.................\n");
		if (0 != imiReadNextFrame(g_streams[0], &pFrameColor, 150))
		{
			printf("read color frame Failed, channel index : %d\n", avStreamIndex++);
			return false;
		}
		
		printf("start copy image color data*************************\n");
		memcpy(imRGB.data, (const void*)pFrameColor->pData, pFrameColor->size);
		printf("end copy image color data*************************\n");

		color_t = pFrameColor->timeStamp;
		g_bColorFrameOK = true;
		ROS_INFO("release pFrameColor");
		imiReleaseFrame(&pFrameColor);
		ROS_INFO("end release pFrameColor");
	}


	if (g_bDepthFrameOK != true)
	{
		if(0 != imiWaitForStreams(g_streams, g_streamNum, &avStreamIndex, -1))
		{
				//usleep(500*1000);
				//return false;
			printf("wait stream fail..............\n");
		}

		if (0 != imiReadNextFrame(g_streams[1], &pFrameDepth, 150))
		{
			printf("read depth Failed, channel index : %d\n", avStreamIndex++);
			return false;
		}

		printf("start copy image depth data*************************\n");
		memcpy(imD.data, pFrameDepth->pData, pFrameDepth->size );
		printf("end copy image depth data*************************\n");

		depth_t = pFrameDepth->timeStamp;
		g_bDepthFrameOK = true;
		ROS_INFO("release pFrameDepth");
		imiReleaseFrame(&pFrameDepth);
		ROS_INFO("end release pFrameDepth");
	}

	printf("start flip image data***********************************\n");
	printf("bColorFrameOK:%d,bDepthFrameOK:%d\n",g_bColorFrameOK,g_bDepthFrameOK);

	if(g_bColorFrameOK && g_bDepthFrameOK) 
	{
		printf("start flip image data***********************************\n");
		cv::flip(imRGB, imRGBFlip, 1);
		cv::flip(imD, imDEPFlip, 1);
		printf("end flip image data***********************************\n");

		g_bColorFrameOK = false;
		g_bDepthFrameOK = false;

		return true;
	}
	else
	{
		return false;
	}
#endif*/

    // wait for stream, -1 means infinite;
    int32_t avStreamIndex;
	if (0 != imiWaitForStreams(g_streams, g_streamNum, &avStreamIndex, -1)) 
	{
		return false;
	}

    // frame coming, read.
	ImiImageFrame* imiFrame = NULL;
    if (0 != imiReadNextFrame(g_streams[avStreamIndex], &imiFrame, 30)) 
	{
		return false;
	}

	if (NULL == imiFrame)
	{
		return true;
	}

    // show to the window
    if (IMI_COLOR_FRAME == imiFrame->type) 
	{
        //uint32_t rgbSize;
        switch(imiFrame->pixelFormat)
		{
			case IMI_PIXEL_FORMAT_IMAGE_H264: 
			case IMI_PIXEL_FORMAT_IMAGE_RGB24:
			{
				memcpy((void*)imRGB.data, (const void*)imiFrame->pData, imiFrame->size);
				//g_realColorImageSize = imiFrame->size;
				break;
			} 
			case IMI_PIXEL_FORMAT_IMAGE_YUV420SP:
			{
				//YUV420SPToRGB((uint8_t*)imRGB.data, (uint8_t*)imiFrame->pData, imiFrame->width, imiFrame->height);
				//g_realColorImageSize = imiFrame->width * imiFrame->height * 3;
				break;
			} 
			default: 
			{
				break;
			}
		}
		g_bColorFrameOK = true;
    } 
	else //IMI_DEPTH_FRAME == imiFrame->type
	{
		/*uint32_t i;
		uint16_t * pde = (uint16_t*)imiFrame->pData;
		for (i = 0; i < imiFrame->size/2; ++i)
		{
			g_depthImage[i].r = pde[i] >> 3;
			g_depthImage[i].g = g_depthImage[i].b = g_depthImage[i].r;
		}
        g_realDepthImageSize = i;
		g_realDepthWidth = imiFrame->width;
		g_realDepthHeight = imiFrame->height;
		g_bDepthFrameOK = true;*/
		
		ROS_INFO("depth->imiFrame->size:%d",imiFrame->size);
		memcpy((void*)imD.data, (void*)imiFrame->pData, imiFrame->size );
    }
			           
	if (g_bColorFrameOK && g_bDepthFrameOK)
	{
		/*g_pRender->initViewPort();

		WindowHint hint(0, 0, IMAGE_WIDTH, IMAGE_HEIGHT);
		g_pRender->draw((uint8_t*)g_colorImage, g_realColorImageSize, hint);

		hint.x += IMAGE_WIDTH;
		hint.w = g_realDepthWidth;
		hint.h = g_realDepthHeight;
		g_pRender->draw((uint8_t*)g_depthImage, g_realDepthImageSize, hint);

		g_pRender->update();

		g_bColorFrameOK = false;
		g_bDepthFrameOK = false;*/
		ROS_INFO("Get image data success.............................");
	}
	
	imiReleaseFrame(&imiFrame);
    return true;
}


void conDepthToGray(cv::Mat& cv_image_depth,cv::Mat& cv_image_gray)
{
	ROS_INFO("cv_image_depth->rows:%d",cv_image_depth.rows);
	ROS_INFO("cv_image_depth->cols:%d",cv_image_depth.cols);

	unsigned short  max = 0;
	for(int i = 0;i < cv_image_depth.rows;i++){
		for(int j = 0;j < cv_image_depth.cols;j++)
		{
			unsigned short value = cv_image_depth.at<unsigned short>(i,j);
			if(value > max)
				max = value;
		}
	}
	ROS_INFO("max:%d",max);
	for(int i = 0;i < cv_image_depth.rows;i++){
		for(int j = 0;j < cv_image_depth.cols;j++)
		{
			unsigned short value = cv_image_depth.at<unsigned short>(i,j);
			unsigned char value_ = (unsigned char)((max-value)/(max*1.0)*255);
			cv_image_gray.at<unsigned char>(i,j) = value_;
		}
	}
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");

  ros::NodeHandle nh_;
  image_transport::ImageTransport it_(nh_);
  image_transport::Publisher image_pub_color_ = it_.advertise("/camera/rgb/image_color", 1);
  image_transport::Publisher image_pub_depth_ = it_.advertise("/camera/depth/image", 1);

	cv::Mat cv_image_color(cv::Size(COLOR_IMAGE_WIDTH,COLOR_IMAGE_HEIGHT),CV_8UC3);
	cv::Mat cv_image_depth(cv::Size(DEPTH_IMAGE_WIDTH,DEPTH_IMAGE_HEIGHT),CV_16UC1);
	cv::Mat cv_image_gray(cv::Size(DEPTH_IMAGE_WIDTH,DEPTH_IMAGE_HEIGHT),CV_8U);

	if(init_camera() == true)
		ROS_INFO("init camera success.....................");
	else{
		ROS_INFO("init camera fail.....................");
		return 0;
	}

  ros::Rate loop_rate(30);
  while(ros::ok()){  

	if(get_img_data(cv_image_color,cv_image_depth) == true){
		ROS_INFO("get img success ........................");
	}
	else{
		ROS_INFO("get img fail......................");
		release_camera();
		continue;
	}

	conDepthToGray(cv_image_depth,cv_image_gray);

  	imshow("cv_image_color",cv_image_color);
  	imshow("cv_image_gray",cv_image_gray);

	cv::waitKey(10);

	ros::Time time = ros::Time::now(); 

  	cv_bridge::CvImage cvi_color;
  	cvi_color.header.stamp = time;
  	cvi_color.header.frame_id = "image_color";
  	cvi_color.encoding = "bgr8";
  	cvi_color.image = cv_image_color;
	
	ROS_INFO("start transfer image color data to ros message");
  	sensor_msgs::Image msg_im_color;
  	cvi_color.toImageMsg(msg_im_color);
	ROS_INFO("end transfer image color data to ros message");
	
  	cv_bridge::CvImage cvi_depth;
  	cvi_depth.header.stamp = time;
  	cvi_depth.header.frame_id = "image_depth";
  	cvi_depth.encoding = "mono16";
  	cvi_depth.image = cv_image_depth;
	
	ROS_INFO("start transfer image depth data to ros message");
	sensor_msgs::Image msg_im_depth;
	cvi_depth.toImageMsg(msg_im_depth);
	ROS_INFO("end transfer image depth data to ros message");

  	image_pub_color_.publish(msg_im_color);
	image_pub_depth_.publish(msg_im_depth);

  	ros::spinOnce();
  	loop_rate.sleep();
  }

  cv_image_color.release();
  cv_image_depth.release();
  cv_image_gray.release();
  release_camera();
  return 0;
}

#ifndef _MSG_CONVERSION_H
#define _MSG_CONVERSION_H

#include "reconstruction3d_nodelet/StereoCameraModel.h"
#include "reconstruction3d_nodelet/Transform.h"

#include <rtabmap/utilite/ULogger.h>

CameraModel cameraModelFromROS(
		const sensor_msgs::CameraInfo & camInfo,
		const Transform & localTransform)
{
	cv::Mat D;
	if(camInfo.D.size())
	{
		D = cv::Mat(1, camInfo.D.size(), CV_64FC1);
		memcpy(D.data, camInfo.D.data(), D.cols*sizeof(double));
	}

	cv:: Mat K;
	UASSERT(camInfo.K.empty() || camInfo.K.size() == 9);
	if(!camInfo.K.empty())
	{
		K = cv::Mat(3, 3, CV_64FC1);
		memcpy(K.data, camInfo.K.elems, 9*sizeof(double));
	}

	cv:: Mat R;
	UASSERT(camInfo.R.empty() || camInfo.R.size() == 9);
	if(!camInfo.R.empty())
	{
		R = cv::Mat(3, 3, CV_64FC1);
		memcpy(R.data, camInfo.R.elems, 9*sizeof(double));
	}

	cv:: Mat P;
	UASSERT(camInfo.P.empty() || camInfo.P.size() == 12);
	if(!camInfo.P.empty())
	{
		P = cv::Mat(3, 4, CV_64FC1);
		memcpy(P.data, camInfo.P.elems, 12*sizeof(double));
	}

	return CameraModel(
			"ros",
			cv::Size(camInfo.width, camInfo.height),
			K, D, R, P,
			localTransform);
}

#endif

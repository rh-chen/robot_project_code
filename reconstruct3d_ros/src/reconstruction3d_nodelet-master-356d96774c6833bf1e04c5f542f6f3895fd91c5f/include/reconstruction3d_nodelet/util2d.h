#ifndef _UTIL2D_H
#define _UTIL2D_H

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

namespace util2d {

    cv::Mat decimate(const cv::Mat & image, int decimation);
    cv::Mat computeDisparity( const cv::Mat & leftImage, const cv::Mat & rightImage);
    cv::Mat disparityFromStereoImages(const cv::Mat & leftImage, const cv::Mat & rightImage);

}
#endif

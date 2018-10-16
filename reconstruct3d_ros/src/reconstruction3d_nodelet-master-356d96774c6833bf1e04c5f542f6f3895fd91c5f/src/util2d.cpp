#include "reconstruction3d_nodelet/util2d.h"

#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UMath.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/utilite/UTimer.h>
#include <rtabmap/utilite/UStl.h>

#include "opencv2/calib3d.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core/utility.hpp"
#include "opencv2/ximgproc/disparity_filter.hpp"

namespace util2d {

    cv::Mat decimate(const cv::Mat & image, int decimation)
    {
        UASSERT(decimation >= 1);
        cv::Mat out;
        if(!image.empty())
        {
            if(decimation > 1)
            {
                if((image.type() == CV_32FC1 || image.type()==CV_16UC1))
                {
                    UASSERT_MSG(image.rows % decimation == 0 && image.cols % decimation == 0, "Decimation of depth images should be exact!");

                    out = cv::Mat(image.rows/decimation, image.cols/decimation, image.type());
                    if(image.type() == CV_32FC1)
                    {
                        for(int j=0; j<out.rows; ++j)
                        {
                            for(int i=0; i<out.cols; ++i)
                            {
                                out.at<float>(j, i) = image.at<float>(j*decimation, i*decimation);
                            }
                        }
                    }
                    else // CV_16UC1
                    {
                        for(int j=0; j<out.rows; ++j)
                        {
                            for(int i=0; i<out.cols; ++i)
                            {
                                out.at<unsigned short>(j, i) = image.at<unsigned short>(j*decimation, i*decimation);
                            }
                        }
                    }
                }
                else
                {
                    cv::resize(image, out, cv::Size(), 1.0f/float(decimation), 1.0f/float(decimation), cv::INTER_AREA);
                }
            }
            else
            {
                out = image;
            }
        }
        return out;
    }

    cv::Mat computeDisparity( const cv::Mat & leftImage, const cv::Mat & rightImage)
    {
        UASSERT(!leftImage.empty() && !rightImage.empty());
        UASSERT(leftImage.cols == rightImage.cols && leftImage.rows == rightImage.rows);
        UASSERT((leftImage.type() == CV_8UC1 || leftImage.type() == CV_8UC3) && rightImage.type() == CV_8UC1);

        cv::Mat leftMono;
        if(leftImage.channels() == 3)
        {
            cv::cvtColor(leftImage, leftMono, CV_BGR2GRAY);
        }
        else
        {
            leftMono = leftImage;
        }
        cv::Mat rightMono;
        if(rightImage.channels() == 3)
        {
            cv::cvtColor(rightImage, rightMono, CV_BGR2GRAY);
        }
        else
        {
            rightMono = rightImage;
        }

        int blockSize_         = 15;  //15
        int minDisparity_      = 0;   //0
        int numDisparities_    = 64;  //64
        int preFilterSize_     = 9;   //9
        int preFilterCap_      = 31;  //31
        int uniquenessRatio_   = 15;  //15
        int textureThreshold_  = 10;  //10
        int speckleWindowSize_ = 100; //100
        int speckleRange_      = 4;   //4

        cv::Mat disparity;
#if CV_MAJOR_VERSION < 3
        cv::StereoBM stereo(cv::StereoBM::BASIC_PRESET);
        stereo.state->SADWindowSize = blockSize_;
        stereo.state->minDisparity = minDisparity_;
        stereo.state->numberOfDisparities = numDisparities_;
        stereo.state->preFilterSize = preFilterSize_;
        stereo.state->preFilterCap = preFilterCap_;
        stereo.state->uniquenessRatio = uniquenessRatio_;
        stereo.state->textureThreshold = textureThreshold_;
        stereo.state->speckleWindowSize = speckleWindowSize_;
        stereo.state->speckleRange = speckleRange_;
        stereo(leftMono, rightImage, disparity, CV_16SC1);
#else
        cv::Ptr<cv::StereoBM> stereo = cv::StereoBM::create();
        stereo->setBlockSize(blockSize_);
        stereo->setMinDisparity(minDisparity_);
        stereo->setNumDisparities(numDisparities_);
        stereo->setPreFilterSize(preFilterSize_);
        stereo->setPreFilterCap(preFilterCap_);
        stereo->setUniquenessRatio(uniquenessRatio_);
        stereo->setTextureThreshold(textureThreshold_);
        stereo->setSpeckleWindowSize(speckleWindowSize_);
        stereo->setSpeckleRange(speckleRange_);
        stereo->compute(leftMono, rightMono, disparity);

//        cv::Ptr<cv::StereoMatcher> right_matcher = cv::ximgproc::createRightMatcher(stereo);
//        cv::Mat right_disp;
//        right_matcher->compute(rightMono, leftMono, right_disp);
//
//        cv::Mat filtered_disp;
//        cv::Ptr<cv::ximgproc::DisparityWLSFilter> wls_filter;
//        wls_filter = cv::ximgproc::createDisparityWLSFilter(stereo);
//        double lambda = 8000.0;
//        double sigma = 1.5;
//        wls_filter->setLambda(lambda);
//        wls_filter->setSigmaColor(sigma);
//        wls_filter->filter(disparity, leftImage, filtered_disp, right_disp);

//        cv::Mat filtered_disp_vis;
//        cv::ximgproc::getDisparityVis(filtered_disp, filtered_disp_vis, 1.0);
//        cv::namedWindow("filtered disparity", WINDOW_AUTOSIZE);
//        cv::imshow("filtered disparity", filtered_disp_vis);
//        cv::waitKey(10);
#endif
        //return filtered_disp;
        return disparity;
    }

    cv::Mat disparityFromStereoImages(const cv::Mat & leftImage, const cv::Mat & rightImage)
    {
        UASSERT(!leftImage.empty() && !rightImage.empty());
        UASSERT(leftImage.cols == rightImage.cols && leftImage.rows == rightImage.rows);
        UASSERT((leftImage.type() == CV_8UC1 || leftImage.type() == CV_8UC3) && rightImage.type() == CV_8UC1);

        cv::Mat leftMono;
        if(leftImage.channels() == 3)
            cv::cvtColor(leftImage, leftMono, CV_BGR2GRAY);
        else
            leftMono = leftImage;

        return computeDisparity(leftMono, rightImage);
    }

}

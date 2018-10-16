#include "reconstruction3d_nodelet/util3d.h"

#include <rtabmap/utilite/UMath.h>
#include <rtabmap/utilite/ULogger.h>

#include "reconstruction3d_nodelet/util2d.h"

namespace util3d {

    bool isFinite(const cv::Point3f & pt)
    {
        return uIsFinite(pt.x) && uIsFinite(pt.y) && uIsFinite(pt.z);
    }

    // inspired from ROS image_geometry/src/stereo_camera_model.cpp
    cv::Point3f projectDisparityTo3D(
            const cv::Point2f & pt,
            float disparity,
            const StereoCameraModel & model)
    {
        if(disparity > 0.0f && model.baseline() > 0.0f && model.left().fx() > 0.0f)
        {
            //Z = baseline * f / (d + cx1-cx0);
            float c = 0.0f;
            if(model.right().cx()>0.0f && model.left().cx()>0.0f)
            {
                c = model.right().cx() - model.left().cx();
            }
            float W = model.baseline()/(disparity + c);
            return cv::Point3f((pt.x - model.left().cx())*W, (pt.y - model.left().cy())*W, model.left().fx()*W);
        }
        float bad_point = std::numeric_limits<float>::quiet_NaN ();
        return cv::Point3f(bad_point, bad_point, bad_point);
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudFromStereoImages(
            const cv::Mat & imageLeft,
            const cv::Mat & imageRight,
            const StereoCameraModel & model,
            int decimation,
            float maxDepth,
            float minDepth,
            std::vector<int> * validIndices)
    {
        UASSERT(!imageLeft.empty() && !imageRight.empty());
        UASSERT(imageRight.type() == CV_8UC1);
        UASSERT(imageLeft.channels() == 3 || imageLeft.channels() == 1);
        UASSERT(imageLeft.rows == imageRight.rows &&
                imageLeft.cols == imageRight.cols);
        UASSERT(decimation >= 1);

        cv::Mat leftColor = imageLeft;
        cv::Mat rightMono = imageRight;

        StereoCameraModel modelDecimation = model;

        if(leftColor.rows % decimation != 0 ||
                leftColor.cols % decimation != 0)
        {
            leftColor = util2d::decimate(leftColor, decimation);
            rightMono = util2d::decimate(rightMono, decimation);
            modelDecimation.scale(1/float(decimation));
            decimation = 1;
        }

        cv::Mat leftMono;
        if(leftColor.channels() == 3)
        {
            cv::cvtColor(leftColor, leftMono, CV_BGR2GRAY);
        }
        else
        {
            leftMono = leftColor;
        }

        return cloudFromDisparityRGB(
                leftColor,
                util2d::disparityFromStereoImages(leftMono, rightMono),
                modelDecimation,
                decimation,
                maxDepth,
                minDepth,
                validIndices);
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudFromDisparityRGB(
            const cv::Mat & imageRgb,
            const cv::Mat & imageDisparity,
            const StereoCameraModel & model,
            int decimation,
            float maxDepth,
            float minDepth,
            std::vector<int> * validIndices)
    {
        UASSERT(!imageRgb.empty() && !imageDisparity.empty());
        UASSERT(imageRgb.rows == imageDisparity.rows &&
                imageRgb.cols == imageDisparity.cols &&
                (imageDisparity.type() == CV_32FC1 || imageDisparity.type()==CV_16SC1));
        UASSERT(imageRgb.channels() == 3 || imageRgb.channels() == 1);
        UASSERT(decimation >= 1);
        UASSERT(imageDisparity.rows % decimation == 0 && imageDisparity.cols % decimation == 0);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

        bool mono;
        if(imageRgb.channels() == 3) // BGR
        {
            mono = false;
        }
        else // Mono
        {
            mono = true;
        }

        //cloud.header = cameraInfo.header;
        cloud->height = imageRgb.rows/decimation;
        cloud->width  = imageRgb.cols/decimation;
        cloud->is_dense = false;
        cloud->resize(cloud->height * cloud->width);
        if(validIndices)
        {
            validIndices->resize(cloud->size());
        }

        int oi=0;
        for(int h = 0; h < imageRgb.rows && h/decimation < (int)cloud->height; h+=decimation)
        {
            for(int w = 0; w < imageRgb.cols && w/decimation < (int)cloud->width; w+=decimation)
            {
                pcl::PointXYZRGB & pt = cloud->at((h/decimation)*cloud->width + (w/decimation));
                if(!mono)
                {
                    pt.b = imageRgb.at<cv::Vec3b>(h,w)[0];
                    pt.g = imageRgb.at<cv::Vec3b>(h,w)[1];
                    pt.r = imageRgb.at<cv::Vec3b>(h,w)[2];
                }
                else
                {
                    unsigned char v = imageRgb.at<unsigned char>(h,w);
                    pt.b = v;
                    pt.g = v;
                    pt.r = v;
                }

                float disp = imageDisparity.type()==CV_16SC1?float(imageDisparity.at<short>(h,w))/16.0f:imageDisparity.at<float>(h,w);
                cv::Point3f ptXYZ = projectDisparityTo3D(cv::Point2f(w, h), disp, model);
                if(util3d::isFinite(ptXYZ) && ptXYZ.z >= minDepth && (maxDepth<=0.0f || ptXYZ.z <= maxDepth))
                {
                    pt.x = ptXYZ.x;
                    pt.y = ptXYZ.y;
                    pt.z = ptXYZ.z;
                    if(validIndices)
                    {
                        validIndices->at(oi++) = (h/decimation)*cloud->width + (w/decimation);
                    }
                }
                else
                {
                    pt.x = pt.y = pt.z = std::numeric_limits<float>::quiet_NaN();
                }
            }
        }
        if(validIndices)
        {
            validIndices->resize(oi);
        }
        return cloud;
    }


    pcl::PointCloud<pcl::PointXYZRGB>::Ptr passThrough(
            const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
            const std::string & axis,
            float min,
            float max,
            bool negative)
    {
        UASSERT(max > min);
        UASSERT(axis.compare("x") == 0 || axis.compare("y") == 0 || axis.compare("z") == 0);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr output(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PassThrough<pcl::PointXYZRGB> filter;
        filter.setNegative(negative);
        filter.setFilterFieldName(axis);
        filter.setFilterLimits(min, max);
        filter.setInputCloud(cloud);
        filter.filter(*output);
        return output;
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr voxelize(
            const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
            const pcl::IndicesPtr & indices,
            float voxelSize)
    {
        UASSERT(voxelSize > 0.0f);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr output(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::VoxelGrid<pcl::PointXYZRGB> filter;
        filter.setLeafSize(voxelSize, voxelSize, voxelSize);
        filter.setInputCloud(cloud);
        if(indices->size())
        {
            filter.setIndices(indices);
        }
        filter.filter(*output);
        return output;
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr voxelize(
            const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
            float voxelSize)
    {
        pcl::IndicesPtr indices(new std::vector<int>);
        return voxelize(cloud, indices, voxelSize);
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr removeNaNFromPointCloud(
            const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr output(new pcl::PointCloud<pcl::PointXYZRGB>);
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*cloud, *output, indices);
        return output;
    }

    pcl::IndicesPtr radiusFiltering(
            const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
            float radiusSearch,
            int minNeighborsInRadius)
    {
        pcl::IndicesPtr indices(new std::vector<int>);
        return radiusFiltering(cloud, indices, radiusSearch, minNeighborsInRadius);
    }

    pcl::IndicesPtr radiusFiltering(
            const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
            const pcl::IndicesPtr & indices,
            float radiusSearch,
            int minNeighborsInRadius)
    {
        pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>(false));

        if(indices->size())
        {
            pcl::IndicesPtr output(new std::vector<int>(indices->size()));
            int oi = 0; // output iterator
            tree->setInputCloud(cloud, indices);
            for(unsigned int i=0; i<indices->size(); ++i)
            {
                std::vector<int> kIndices;
                std::vector<float> kDistances;
                int k = tree->radiusSearch(cloud->at(indices->at(i)), radiusSearch, kIndices, kDistances);
                if(k > minNeighborsInRadius)
                {
                    output->at(oi++) = indices->at(i);
                }
            }
            output->resize(oi);
            return output;
        }
        else
        {
            pcl::IndicesPtr output(new std::vector<int>(cloud->size()));
            int oi = 0; // output iterator
            tree->setInputCloud(cloud);
            for(unsigned int i=0; i<cloud->size(); ++i)
            {
                std::vector<int> kIndices;
                std::vector<float> kDistances;
                int k = tree->radiusSearch(cloud->at(i), radiusSearch, kIndices, kDistances);
                if(k > minNeighborsInRadius)
                {
                    output->at(oi++) = i;
                }
            }
            output->resize(oi);
            return output;
        }
    }

}

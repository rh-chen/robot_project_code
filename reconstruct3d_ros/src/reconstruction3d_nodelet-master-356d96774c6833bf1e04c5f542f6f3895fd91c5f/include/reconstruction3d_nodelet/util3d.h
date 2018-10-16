#ifndef _UTIL3D_H
#define _UTIL3D_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/pcl_base.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/frustum_culling.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/crop_box.h>

#include <pcl/features/normal_3d_omp.h>

#include <pcl/search/kdtree.h>

#include <pcl/common/common.h>

#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UMath.h>
#include <rtabmap/utilite/UConversion.h>

#include <opencv2/core/core.hpp>
#include <map>
#include <list>
#include <vector>

#include "reconstruction3d_nodelet/StereoCameraModel.h"

namespace util3d {

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudFromDisparityRGB(
            const cv::Mat & imageRgb,
            const cv::Mat & imageDisparity,
            const StereoCameraModel & model,
            int decimation = 1,
            float maxDepth = 0.0f,
            float minDepth = 0.0f,
            std::vector<int> * validIndices = 0);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudFromStereoImages(
            const cv::Mat & imageLeft,
            const cv::Mat & imageRight,
            const StereoCameraModel & model,
            int decimation = 1,
            float maxDepth = 0.0f,
            float minDepth = 0.0f,
            std::vector<int> * validIndices = 0);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr passThrough(
            const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
            const std::string & axis,
            float min,
            float max,
            bool negative = true);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr voxelize(
            const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
            const pcl::IndicesPtr & indices,
            float voxelSize);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr voxelize(
            const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
            float voxelSize);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr removeNaNFromPointCloud(
            const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud);

    pcl::IndicesPtr radiusFiltering(
            const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
            float radiusSearch,
            int minNeighborsInRadius);

    pcl::IndicesPtr radiusFiltering(
            const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
            const pcl::IndicesPtr & indices,
            float radiusSearch,
            int minNeighborsInRadius);
}

#endif

#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <string>
#include <memory>
#include <unistd.h>
#include <nodelet/loader.h>



int main(int argc, char* argv[])
{
    ros::init(argc, argv, "DepthImageToPointCloud");

    nodelet::Loader nodelet;
    nodelet::M_string remap(ros::names::getRemappings());
    nodelet::V_string nargv;
    nodelet.load(ros::this_node::getName(),
            "depth_image_proc/point_cloud_xyzrgb",
            remap, nargv);

    ros::spin();

    return 0;
}

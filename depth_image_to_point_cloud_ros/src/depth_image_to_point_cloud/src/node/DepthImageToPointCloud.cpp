#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <string>
#include <memory>
#include <unistd.h>
#include <nodelet/loader.h>



int main(int argc, char* argv[])
{
    ros::init(argc, argv, "depth_image_to_point_cloud");

    nodelet::Loader nodelet;
    nodelet::M_string remap(ros::names::getRemappings());
    nodelet::V_string nargv;
    nodelet.load(ros::this_node::getName(),
            "depth_image_to_point_cloud/PointCloudXyzrgbNodelet",
            remap, nargv);

    ros::spin();

    return 0;
}

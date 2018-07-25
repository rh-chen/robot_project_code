#include <ros/ros.h>
#include "InuSensor.h"
#include "DepthStream.h"

#include <iostream>
#include <fstream>
#include <string>
#include <memory>
#include <unistd.h>

#include <nodelet/loader.h>


int main(int argc, char* argv[])
{
    ros::init(argc, argv, "inuitive_ros_wrapper");

    nodelet::Loader nodelet;

    nodelet::M_string remap(ros::names::getRemappings());

    nodelet::V_string nargv;

    nodelet.load(ros::this_node::getName(),
            "inuitive_ros_wrapper/InuitiveRosWrapperNodelet",
            remap, nargv);

    ros::spin();

    return 0;
}

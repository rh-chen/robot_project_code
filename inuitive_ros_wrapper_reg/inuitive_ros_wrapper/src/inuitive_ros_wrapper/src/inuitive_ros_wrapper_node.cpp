#include <ros/ros.h>
#include "InuSensor.h"
#include "DepthStream.h"

#include <iostream>
#include <fstream>
#include <string>
#include <memory>

#ifdef _MSC_VER
#include <windows.h>
#else
#include <unistd.h>
#endif

#include <nodelet/loader.h>



int main(int argc, char* argv[])
{
    /*std::cout << "Inuitive sample application - saving Depth images to disk" << std::endl;
    CTest test;
    return test.Execute();*/

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

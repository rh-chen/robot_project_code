#include <ros/ros.h>
#include <nodelet/loader.h>



int main(int argc, char* argv[])
{
    ros::init(argc, argv, "line_detection_and_rotation");

    nodelet::Loader nodelet;
    nodelet::M_string remap(ros::names::getRemappings());
    nodelet::V_string nargv;
    nodelet.load(ros::this_node::getName(),
            "line_detection_and_rotation/LineDetectionAndRotationNodelet",
            remap, nargv);

    ros::spin();

    return 0;
}

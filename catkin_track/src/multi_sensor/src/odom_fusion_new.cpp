#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <kobuki_msgs/CliffEvent.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_datatypes.h>
#include <kobuki_msgs/WheelDropEvent.h>


int main(int argc, char** argv){

    ros::init(argc, argv, "odometry_sub");

    ros::NodeHandle n;

    tf::Transform transform1,transform2,transform3;

    tf::Quaternion q;

    transform1.setOrigin( tf::Vector3(1,2,0) );
    q.setRPY(0,0,0);
    transform1.setRotation(q);


    transform2.setOrigin( tf::Vector3(2,3,0) );
    q.setRPY(0,0,0);
    transform2.setRotation(q);


    transform3 = transform2 * transform2;

  // transform3 =  transform1.inverseTimes(transform2);

   // transform2 = transform1 * transform3;
    ros::Rate r(30.0);


    while(n.ok()){

       // std::cout <<transform1.getOrigin().getX() << transform1.getOrigin().getY() << transform1.getOrigin().getZ() << transform1.getRotation().getX() << transform1.getRotation().getY() << transform1.getRotation().getZ() << transform1.getRotation().getW()  <<std::endl;
        std::cout << transform3.getOrigin().getX() <<" "<< transform3.getOrigin().getY()<<std::endl;
        ros::spinOnce();
        r.sleep();
        }


      return 0;
}

#include "ros/ros.h"
#include "ros/time.h"
#include "sensor_msgs/Range.h"
#include "visualization_msgs/Marker.h"
#include <tf/transform_broadcaster.h>
visualization_msgs::Marker points,line;

void rvizCallBack(const geometry_msgs::PointStamped::ConstPtr& msg)
{
    geometry_msgs::Point p;

    p.x=msg->point.x;
    p.y=msg->point.y;
    p.z=msg->point.z;

    std::cout << p.x<< " "<< p.y<<  std::endl;
    points.points.push_back(p);

}


int main(int argc, char **argv)
{
      ros::init(argc, argv, "global_rrt_frontier_detector");
      ros::NodeHandle nh;
      ros::Subscriber rviz_sub= nh.subscribe("/clicked_point", 100 ,rvizCallBack);
      ros::Publisher pub = nh.advertise<visualization_msgs::Marker>("shapes", 10);
      ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);

      visualization_msgs::Marker line_list;
      ros::Rate rate(60);

      while(ros::ok())
      {

      line_list.header.frame_id = "my_frame";
      line_list.header.stamp = ros::Time::now();
      line_list.ns = "lines";
      line_list.action = visualization_msgs::Marker::ADD;
      line_list.pose.orientation.w = 1.0;
      line_list.id = 2;
      line_list.type = visualization_msgs::Marker::LINE_LIST;
      line_list.scale.x = 0.1;
      line_list.color.r = 1.0;
      line_list.color.a = 1.0;
      if(points.points.size() > 0 )
      line_list.points.push_back(points.points[0]);
      if(points.points.size() > 1 )
      line_list.points.push_back(points.points[1]);

      marker_pub.publish(line_list);
      pub.publish(points) ;
      points.points.clear();

      static tf::TransformBroadcaster br;
      tf::Transform transform;
      transform.setOrigin( tf::Vector3(0, 0, 0) );
      tf::Quaternion q;
      q.setRPY(0, 0,0);
       transform.setRotation(q);
       br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "my_frame"));

       ros::spinOnce();
       rate.sleep();

      }
      return 0;
}

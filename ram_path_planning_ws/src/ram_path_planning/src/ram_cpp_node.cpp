#include <nav_msgs/GetMap.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/matx.hpp>
#include <sys/stat.h>

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/distortion_models.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PointStamped.h>
#include <image_transport/image_transport.h>

#include <boost/thread/thread.hpp>
#include <boost/make_shared.hpp>

#include <stdio.h>
#include <math.h>

#include <hash_set>
#include <set>
#include <map>
#include <algorithm> 

#include <tf/tf.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "ram_path_planning/ModifyMap.h"
#include "ram_path_planning/MapRotate.h"
#include "ram_path_planning/Cpp.h"

class darp_test{
public:
	ros::NodeHandle n;
	ros::Subscriber sub;

	darp_test(ros::NodeHandle n_):n(n_){
		sub = n.subscribe("/clicked_point",1000,&darp_test::clickCallBack,this);
	}

	visualization_msgs::Marker createMarker(const std::string markerName,
									uint32_t type, 
									geometry_msgs::Pose pose, 
									geometry_msgs::Vector3 scale, 
									std_msgs::ColorRGBA color,
									int32_t id, 
									std::string frame_id = std::string("s_map"))
	{

			//marker start point
			visualization_msgs::Marker marker;
			marker.header.frame_id = frame_id;
			marker.header.stamp = ros::Time();
			marker.ns = "marker_" + markerName;
			marker.id = id;
			marker.type = type;
			marker.action = visualization_msgs::Marker::ADD;
			marker.pose.position.x = pose.position.x;
			marker.pose.position.y = pose.position.y;
			marker.pose.position.z = pose.position.z;
			marker.pose.orientation.x = pose.orientation.x;
			marker.pose.orientation.y = pose.orientation.y;
			marker.pose.orientation.z = pose.orientation.z;
			marker.pose.orientation.w = pose.orientation.w;
			marker.scale.x = scale.x;
			marker.scale.y = scale.y;
			marker.scale.z = scale.z;
			marker.color.a = color.a;
			marker.color.r = color.r;
			marker.color.g = color.g;
			marker.color.b = color.b;

			return marker;
	}


	void clickCallBack(const geometry_msgs::PointStamped::ConstPtr &msg)
	{
            ros::Publisher pub_map_modify = n.advertise<nav_msgs::OccupancyGrid>("/modify_static_map",1);
			ros::Publisher pub_map_rotate = n.advertise<nav_msgs::OccupancyGrid>("/map_rotate_map",1);

            ros::ServiceClient map_modify_client = n.serviceClient<ram_path_planning::ModifyMap>("/sweeper/map_modify_srv");
			ros::ServiceClient map_rotate_client = n.serviceClient<ram_path_planning::MapRotate>("/sweeper/map_rotate_srv");
			ros::ServiceClient mapClient = n.serviceClient<nav_msgs::GetMap>("/static_map");

			ram_path_planning::MapRotate map_rotate_srv;


            ram_path_planning::ModifyMap map_modify_srv;
            map_modify_srv.request.threshold = 95;

			nav_msgs::GetMap getMapSrv;

			ros::Duration(2).sleep();
			if (mapClient.call(getMapSrv)) {
                map_modify_srv.request.map = getMapSrv.response.map;
				std::cout << "map frame_id: " << map_modify_srv.request.map.header.frame_id << std::endl;
			} else {
				    ROS_ERROR("Failed to call map_modify_srv service.");
			}

            ros::Time begin2 = ros::Time::now();
            bool res_srv_map_modify = map_modify_client.call(map_modify_srv);
            ros::Time end2 = ros::Time::now();
	        std::cout << "call map_modify_srv time cost:" << (end2-begin2).toSec() << std::endl;

            if(res_srv_map_modify){
                map_rotate_srv.request.map = map_modify_srv.response.map;
            }else{
                ROS_ERROR("Failed to call map_modify_srv service.");
            }

			ros::Time begin3 = ros::Time::now();
			bool res_map_rotate = map_rotate_client.call(map_rotate_srv);
            ros::Time end3 = ros::Time::now();
            std::cout << "map rotate cost time:" << (end3-begin3).toSec() << std::endl;
            
			if(res_map_rotate){
				ROS_INFO("map rotate success...");
				//add zigzag algorithm
			}
			else
				ROS_INFO("map rotate fail...");

			if (res_map_rotate) {
					ros::ServiceClient client_zigzag = \
                                       n.serviceClient<ram_path_planning::Cpp>("/sweeper/zigzag_cpp_srv");

					ram_path_planning::Cpp srv_zigzag;

					srv_zigzag.request.occupancy_threshold = 95;
					srv_zigzag.request.number_of_layers = 1;
					srv_zigzag.request.map_origin_x = map_rotate_srv.response.map_origin_x;
					srv_zigzag.request.map_origin_y = map_rotate_srv.response.map_origin_y;
					srv_zigzag.request.map_resolution = map_rotate_srv.response.map_resolution;
					srv_zigzag.request.height_between_layers = 1;
					srv_zigzag.request.deposited_material_width = 0.3;
					srv_zigzag.request.contours_filtering_tolerance = 0.1;
					srv_zigzag.request.transform = map_rotate_srv.response.transform;
					srv_zigzag.request.map = map_rotate_srv.response.map;
					srv_zigzag.request.external_contour_threshold = 48;

					ros::Time begin = ros::Time::now();
					bool res_srv_zigzag = client_zigzag.call(srv_zigzag);
					ros::Time end = ros::Time::now();

					std::cout << "call srv_zigzag time cost:" << (end-begin).toSec() << std::endl;
					if(res_srv_zigzag){
							ROS_INFO("call /sweeper/zigzag_cpp service success...");

							ros::Publisher marker_pub = n.advertise<visualization_msgs::MarkerArray>("/cleanner_planner", 1);
							ros::Rate loop_rate(10);
							while(ros::ok()){

                                    pub_map_modify.publish(map_modify_srv.response.map);
									pub_map_rotate.publish(map_rotate_srv.response.map);
#if 0
									int path_size = srv_darp.response.plan.poses.size();

									visualization_msgs::MarkerArray markerArray;

									geometry_msgs::PoseStamped startPose = srv_darp.response.plan.poses[0];

									//marker start pose
									geometry_msgs::Pose  plannerStartPose;
									plannerStartPose.position.x = startPose.pose.position.x;
									plannerStartPose.position.y = startPose.pose.position.y;
									plannerStartPose.orientation.w = 1.0;

									geometry_msgs::Vector3 plannerStartScale;
									plannerStartScale.x = 0.2;
									plannerStartScale.y = 0.2;
									plannerStartScale.z = 0.2;

									std_msgs::ColorRGBA plannerStartColor;
									plannerStartColor.a = 1.0;
									plannerStartColor.g = 1.0;

									int32_t plannerStartId = 0;

									visualization_msgs::Marker markerSphereStart = createMarker("PlannerStart",
																	visualization_msgs::Marker::SPHERE,
																	plannerStartPose,
																	plannerStartScale,
																	plannerStartColor,
																	plannerStartId,
																	srv_darp.request.map.header.frame_id);

									markerArray.markers.push_back(markerSphereStart);

									geometry_msgs::Point last_point;
									last_point.x = startPose.pose.position.x;
									last_point.y = startPose.pose.position.y;

									for(int i = 1; i < path_size; ++i) {

											geometry_msgs::PoseStamped pose = srv_darp.response.plan.poses[i];
											//ROS_INFO_STREAM("poses:%s" << pose);

											//marker planner pose
											geometry_msgs::Pose  markerArrowPose;
											markerArrowPose.position.x = last_point.x;
											markerArrowPose.position.y = last_point.y;
											markerArrowPose.position.z = 0;
											markerArrowPose.orientation.w = 1.0;

											geometry_msgs::Vector3 markerArrowScale;
											markerArrowScale.x = 0.05;
											markerArrowScale.y = 0.1;
											markerArrowScale.z = 0.1;

											std_msgs::ColorRGBA markerArrowColor;
											markerArrowColor.a = 1.0;
											markerArrowColor.r = 1.0;

											int32_t markerArrowId = i;

											visualization_msgs::Marker markerArrow = createMarker("markerArrow",
																			visualization_msgs::Marker::ARROW,
																			markerArrowPose,
																			markerArrowScale,
																			markerArrowColor,
																			markerArrowId,
																			srv_darp.request.map.header.frame_id);

											//arrowHead, arrowEnd
											geometry_msgs::Point p;
											p.x = pose.pose.position.x;
											p.y = pose.pose.position.y;
											p.z = 0;

											geometry_msgs::Point arrowHeadPoint;
											arrowHeadPoint.x = p.x - last_point.x;
											arrowHeadPoint.y = p.y - last_point.y;
											arrowHeadPoint.z = 0;


											geometry_msgs::Point arrowEndPoint;
											arrowEndPoint.x = 0;
											arrowEndPoint.y = 0;
											arrowEndPoint.z = 0;

											markerArrow.points.push_back(arrowEndPoint);
											markerArrow.points.push_back(arrowHeadPoint);

											markerArray.markers.push_back(markerArrow);

											last_point = p;

									}

									marker_pub.publish(markerArray);
#endif 
									ros::spinOnce();
									loop_rate.sleep();
							}
					}
					else
							ROS_ERROR("Failed to call service /sweeper/zigzag_cpp_srv");
			} else {
							ROS_ERROR("Failed to call service /sweeper/map_rotate_srv");
			}
	}
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "scale_map_client");
	
	ros::NodeHandle n;
	
	darp_test dt(n);
	ros::spin();

	return 0;
}

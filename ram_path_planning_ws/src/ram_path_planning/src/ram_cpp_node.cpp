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
#include <nav_msgs/OccupancyGrid.h>
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


class cpp_test{
public:
	ros::NodeHandle n;
	ros::Subscriber sub;
	ros::Subscriber sub_;
	nav_msgs::OccupancyGrid msg_;

	cpp_test(ros::NodeHandle n_):n(n_){
		sub = n.subscribe("/clicked_point",1000,&cpp_test::clickCallBack,this);
		sub_ = n.subscribe("/map",1000,&cpp_test::mapCallBack,this);
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

	void mapCallBack(const nav_msgs::OccupancyGrid::ConstPtr &msg){
		msg_ = *msg;
	}
	void clickCallBack(const geometry_msgs::PointStamped::ConstPtr &msg)
	{
            ros::Publisher pub_map_modify = n.advertise<nav_msgs::OccupancyGrid>("/modify_static_map",1);
			ros::Publisher pub_map_cpp = n.advertise<nav_msgs::OccupancyGrid>("/map_cpp_map",1);

            ros::ServiceClient map_modify_client = n.serviceClient<ram_path_planning::ModifyMap>("/sweeper/map_modify_srv");
			//ros::ServiceClient map_rotate_client = n.serviceClient<ram_path_planning::MapRotate>("/sweeper/map_rotate_srv");
			ros::ServiceClient mapClient = n.serviceClient<nav_msgs::GetMap>("/static_map");

			ram_path_planning::MapRotate map_rotate_srv;


            ram_path_planning::ModifyMap map_modify_srv;
            map_modify_srv.request.threshold = 95;
            map_modify_srv.request.start_position_x = msg->point.x;
			map_modify_srv.request.start_position_y = msg->point.y;
			
            nav_msgs::GetMap getMapSrv;

			ros::Duration(2).sleep();
			//map_modify_srv.request.map = msg_;
			if (mapClient.call(getMapSrv)) {
                map_modify_srv.request.map = getMapSrv.response.map;
				std::cout << "map frame_id: " << map_modify_srv.request.map.header.frame_id << std::endl;
			} else {
				    ROS_ERROR("Failed to call map_modify_srv service.");
					return;
			}

            ros::Time begin2 = ros::Time::now();
            bool res_map_modify = map_modify_client.call(map_modify_srv);
            ros::Time end2 = ros::Time::now();
	        std::cout << "call map_modify_srv time cost:" << (end2-begin2).toSec() << std::endl;

            /*if(res_map_modify){
                map_rotate_srv.request.map = map_modify_srv.response.map;
            }else{
                ROS_ERROR("Failed to call map_modify_srv service.");
				return;
            }*/

			/*ros::Time begin3 = ros::Time::now();
			bool res_map_rotate = map_rotate_client.call(map_rotate_srv);
            ros::Time end3 = ros::Time::now();
            std::cout << "map rotate cost time:" << (end3-begin3).toSec() << std::endl;
            */
			if (res_map_modify) {
					ros::ServiceClient client_zigzag = \
                                       n.serviceClient<ram_path_planning::Cpp>("/sweeper/zigzag_cpp_srv");

					ram_path_planning::Cpp srv_zigzag;

					srv_zigzag.request.occupancy_threshold = 95;
					srv_zigzag.request.number_of_layers = 1;
					srv_zigzag.request.map_origin_x = map_modify_srv.response.map.info.origin.position.x;
					srv_zigzag.request.map_origin_y = map_modify_srv.response.map.info.origin.position.y;
					srv_zigzag.request.map_resolution = map_modify_srv.response.map.info.resolution;
					srv_zigzag.request.start_position_x = msg->point.x;
					srv_zigzag.request.start_position_y = msg->point.y;
					srv_zigzag.request.height_between_layers = 1;
					srv_zigzag.request.deposited_material_width = 0.3;
					srv_zigzag.request.contours_filtering_tolerance = 0.5;
					srv_zigzag.request.map = map_modify_srv.response.map;
					srv_zigzag.request.internal_contour_threshold = 64;

					ros::Time begin = ros::Time::now();
					bool res_srv_zigzag = client_zigzag.call(srv_zigzag);
					ros::Time end = ros::Time::now();

					std::cout << "call srv_zigzag time cost:" << (end-begin).toSec() << std::endl;
					if(res_srv_zigzag){
							ROS_INFO("call /sweeper/zigzag_cpp service success...");

						    ROS_INFO("srv_zigzag.response.polygon.size:%d",srv_zigzag.response.polygon.size());
							ros::Publisher marker_pub = n.advertise<visualization_msgs::MarkerArray>("/cleanner_planner", 1);
							ros::Rate loop_rate(10);
							while(ros::ok()){

                                    pub_map_modify.publish(map_modify_srv.response.map);
									pub_map_cpp.publish(srv_zigzag.response.cv_map);

									visualization_msgs::MarkerArray markerArray;

									int32_t plannerStartId = 0;
#if 1
								//divide polygon
							{
							    //srv_zigzag.response.polygon.size()
                                for(int j = 0;j < srv_zigzag.response.polygon.size();j++){
									geometry_msgs::Polygon polygon_ = srv_zigzag.response.polygon[j];
									geometry_msgs::Pose startPose;
									startPose.position.x = polygon_.points[0].x;
									startPose.position.y = polygon_.points[0].y;
									startPose.position.z = polygon_.points[0].z;

									//marker start pose
									geometry_msgs::Pose  plannerStartPose;
									plannerStartPose.position.x = startPose.position.x;
									plannerStartPose.position.y = startPose.position.y;
									plannerStartPose.orientation.w = 1.0;

									geometry_msgs::Vector3 plannerStartScale;
									plannerStartScale.x = 0.1;
									plannerStartScale.y = 0.1;
									plannerStartScale.z = 0.1;

									std_msgs::ColorRGBA plannerStartColor;
									plannerStartColor.a = 1.0;
									plannerStartColor.r = 1.0;


									visualization_msgs::Marker markerSphereStart = createMarker("PlannerStart",
																	visualization_msgs::Marker::SPHERE,
																	plannerStartPose,
																	plannerStartScale,
																	plannerStartColor,
																	plannerStartId,
																	srv_zigzag.request.map.header.frame_id);
									plannerStartId++;
									markerArray.markers.push_back(markerSphereStart);

									geometry_msgs::Point last_point;
									last_point.x = startPose.position.x;
									last_point.y = startPose.position.y;
									//ROS_INFO("polygon__points_size:%d",polygon_.points.size());
									for(int i = 1; i < polygon_.points.size(); ++i) {

											geometry_msgs::Pose pose;
											pose.position.x = polygon_.points[i].x;
											pose.position.y = polygon_.points[i].y;
											pose.position.z = polygon_.points[i].z;
											//ROS_INFO_STREAM(pose);

											//marker planner pose
											geometry_msgs::Pose  markerArrowPose;
											markerArrowPose.position.x = last_point.x;
											markerArrowPose.position.y = last_point.y;
											markerArrowPose.position.z = 0;
											markerArrowPose.orientation.w = 1.0;

											geometry_msgs::Vector3 markerArrowScale;
											markerArrowScale.x = 0.05;
											markerArrowScale.y = 0.05;
											markerArrowScale.z = 0.05;

											std_msgs::ColorRGBA markerArrowColor;
											markerArrowColor.a = 1.0;
											markerArrowColor.g = 1.0;

											visualization_msgs::Marker markerArrow = createMarker("markerArrow",
																			visualization_msgs::Marker::ARROW,
																			markerArrowPose,
																			markerArrowScale,
																			markerArrowColor,
																			plannerStartId,
																			srv_zigzag.request.map.header.frame_id);

											plannerStartId++;

											//arrowHead, arrowEnd
											geometry_msgs::Point p;
											p.x = pose.position.x;
											p.y = pose.position.y;
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
									//start point to end point
									if(true)
									{
										geometry_msgs::Pose  markerArrowPose;
										markerArrowPose.position.x = last_point.x;
										markerArrowPose.position.y = last_point.y;
										markerArrowPose.position.z = 0;
										markerArrowPose.orientation.w = 1.0;

										geometry_msgs::Vector3 markerArrowScale;
										markerArrowScale.x = 0.05;
										markerArrowScale.y = 0.05;
										markerArrowScale.z = 0.05;

										std_msgs::ColorRGBA markerArrowColor;
										markerArrowColor.a = 1.0;
										markerArrowColor.g = 1.0;


										visualization_msgs::Marker markerArrow = createMarker("markerArrow",
																			visualization_msgs::Marker::ARROW,
																			markerArrowPose,
																			markerArrowScale,
																			markerArrowColor,
																			plannerStartId,
																			srv_zigzag.request.map.header.frame_id);
										plannerStartId++;
											//arrowHead, arrowEnd
										geometry_msgs::Point p;
										p.x = startPose.position.x;
										p.y = startPose.position.y;
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
									}
								}
							}
#endif

#if 1
								//polygon test
							{
                                //ROS_INFO_STREAM("srv_zigzag.response.polygon_test.size:" << srv_zigzag.response.polygon_test.size());
                                for(int j = 0;j < srv_zigzag.response.polygon_test.size();j++){
                                    //std::cout << __FILE__ << __LINE__ << std::endl;
                                    //ROS_INFO_STREAM("J:" << j);
									geometry_msgs::Polygon polygon_ = srv_zigzag.response.polygon_test[j];
									geometry_msgs::Pose startPose;
									startPose.position.x = polygon_.points[0].x;
									startPose.position.y = polygon_.points[0].y;
									startPose.position.z = polygon_.points[0].z;

									//marker start pose
									geometry_msgs::Pose  plannerStartPose;
									plannerStartPose.position.x = startPose.position.x;
									plannerStartPose.position.y = startPose.position.y;
									plannerStartPose.orientation.w = 1.0;

									geometry_msgs::Vector3 plannerStartScale;
									plannerStartScale.x = 0.1;
									plannerStartScale.y = 0.1;
									plannerStartScale.z = 0.1;

									std_msgs::ColorRGBA plannerStartColor;
									plannerStartColor.a = 1.0;
									plannerStartColor.r = 1.0;


									visualization_msgs::Marker markerSphereStart = createMarker("PlannerStart",
																	visualization_msgs::Marker::SPHERE,
																	plannerStartPose,
																	plannerStartScale,
																	plannerStartColor,
																	plannerStartId,
																	srv_zigzag.request.map.header.frame_id);
									plannerStartId++;
									markerArray.markers.push_back(markerSphereStart);

									geometry_msgs::Point last_point;
									last_point.x = startPose.position.x;
									last_point.y = startPose.position.y;
									//ROS_INFO("polygon__points_size:%d",polygon_.points.size());
                                    //std::cout << __FILE__ << __LINE__ << std::endl;
									for(int i = 1; i < polygon_.points.size(); ++i) {

											geometry_msgs::Pose pose;
											pose.position.x = polygon_.points[i].x;
											pose.position.y = polygon_.points[i].y;
											pose.position.z = polygon_.points[i].z;
											//ROS_INFO_STREAM(pose);

											//marker planner pose
											geometry_msgs::Pose  markerArrowPose;
											markerArrowPose.position.x = last_point.x;
											markerArrowPose.position.y = last_point.y;
											markerArrowPose.position.z = 0;
											markerArrowPose.orientation.w = 1.0;

											geometry_msgs::Vector3 markerArrowScale;
											markerArrowScale.x = 0.05;
											markerArrowScale.y = 0.05;
											markerArrowScale.z = 0.05;

											std_msgs::ColorRGBA markerArrowColor;
											markerArrowColor.a = 1.0;
											markerArrowColor.g = 1.0;

											visualization_msgs::Marker markerArrow = createMarker("markerArrow",
																			visualization_msgs::Marker::ARROW,
																			markerArrowPose,
																			markerArrowScale,
																			markerArrowColor,
																			plannerStartId,
																			srv_zigzag.request.map.header.frame_id);

											plannerStartId++;

											//arrowHead, arrowEnd
											geometry_msgs::Point p;
											p.x = pose.position.x;
											p.y = pose.position.y;
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
									//start point to end point
                                    //std::cout << __FILE__ << __LINE__ << std::endl;
									if(true)
									{
										geometry_msgs::Pose  markerArrowPose;
										markerArrowPose.position.x = last_point.x;
										markerArrowPose.position.y = last_point.y;
										markerArrowPose.position.z = 0;
										markerArrowPose.orientation.w = 1.0;

										geometry_msgs::Vector3 markerArrowScale;
										markerArrowScale.x = 0.05;
										markerArrowScale.y = 0.05;
										markerArrowScale.z = 0.05;

										std_msgs::ColorRGBA markerArrowColor;
										markerArrowColor.a = 1.0;
										markerArrowColor.g = 1.0;


										visualization_msgs::Marker markerArrow = createMarker("markerArrow",
																			visualization_msgs::Marker::ARROW,
																			markerArrowPose,
																			markerArrowScale,
																			markerArrowColor,
																			plannerStartId,
																			srv_zigzag.request.map.header.frame_id);
										plannerStartId++;
											//arrowHead, arrowEnd
										geometry_msgs::Point p;
										p.x = startPose.position.x;
										p.y = startPose.position.y;
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
									}
                                    //std::cout << __FILE__ << __LINE__ << std::endl;
								}
							}
#endif


#if 0
								//external contour
								{
									geometry_msgs::Pose startPose = srv_zigzag.response.pose[0];

									//marker start pose
									geometry_msgs::Pose  plannerStartPose;
									plannerStartPose.position.x = startPose.position.x;
									plannerStartPose.position.y = startPose.position.y;
									plannerStartPose.orientation.w = 1.0;

									geometry_msgs::Vector3 plannerStartScale;
									plannerStartScale.x = 0.2;
									plannerStartScale.y = 0.2;
									plannerStartScale.z = 0.2;

									std_msgs::ColorRGBA plannerStartColor;
									plannerStartColor.a = 1.0;
									plannerStartColor.r = 1.0;


									visualization_msgs::Marker markerSphereStart = createMarker("PlannerStart",
																	visualization_msgs::Marker::SPHERE,
																	plannerStartPose,
																	plannerStartScale,
																	plannerStartColor,
																	plannerStartId,
																	srv_zigzag.request.map.header.frame_id);
									plannerStartId++;

									markerArray.markers.push_back(markerSphereStart);

									geometry_msgs::Point last_point;
									last_point.x = startPose.position.x;
									last_point.y = startPose.position.y;

									for(int i = 1; i < srv_zigzag.response.pose.size(); ++i) {

											geometry_msgs::Pose pose = srv_zigzag.response.pose[i];
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
																			plannerStartId++,
																			srv_zigzag.request.map.header.frame_id);

											//arrowHead, arrowEnd
											geometry_msgs::Point p;
											p.x = pose.position.x;
											p.y = pose.position.y;
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
									if(true)
									{
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

										int32_t markerArrowId = srv_zigzag.response.pose.size();

										visualization_msgs::Marker markerArrow = createMarker("markerArrow",
																			visualization_msgs::Marker::ARROW,
																			markerArrowPose,
																			markerArrowScale,
																			markerArrowColor,
																			plannerStartId,
																			srv_zigzag.request.map.header.frame_id);

											//arrowHead, arrowEnd
										geometry_msgs::Point p;
										p.x = startPose.position.x;
										p.y = startPose.position.y;
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
									}
								}
#endif 
#if 1
								//k_nearest
								{
									for(int i = 0; i < srv_zigzag.response.point_nearest.size(); ++i) {

                                        geometry_msgs::Pose keyPose = srv_zigzag.response.point_nearest[i];
                                        //ROS_INFO_STREAM("start pose:(" << startPose.position.x << "," << startPose.position.y << ")");
                                        //marker start pose
                                        geometry_msgs::Pose  keySkeletonPose;
                                        keySkeletonPose.position.x = keyPose.position.x;
                                        keySkeletonPose.position.y = keyPose.position.y;
                                        keySkeletonPose.orientation.w = 1.0;

                                        geometry_msgs::Vector3 keySkeletonScale;
                                        keySkeletonScale.x = 0.1;
                                        keySkeletonScale.y = 0.1;
                                        keySkeletonScale.z = 0.1;

                                        std_msgs::ColorRGBA keySkeletonColor;
                                        keySkeletonColor.a = 1.0;
                                        keySkeletonColor.r = 1.0;


                                        visualization_msgs::Marker markerSphere = createMarker("KeyStraightSkeleton",
                                                                        visualization_msgs::Marker::SPHERE,
                                                                        keySkeletonPose,
                                                                        keySkeletonScale,
                                                                        keySkeletonColor,
                                                                        plannerStartId,
                                                                        srv_zigzag.request.map.header.frame_id);
                                        plannerStartId++;

                                        markerArray.markers.push_back(markerSphere);
									}
									/*if(true)
									{
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

										int32_t markerArrowId = srv_zigzag.response.point_skeleton.size();

										visualization_msgs::Marker markerArrow = createMarker("markerArrow",
																			visualization_msgs::Marker::ARROW,
																			markerArrowPose,
																			markerArrowScale,
																			markerArrowColor,
																			plannerStartId,
																			srv_zigzag.request.map.header.frame_id);

											//arrowHead, arrowEnd
										geometry_msgs::Point p;
										p.x = startPose.position.x;
										p.y = startPose.position.y;
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
									}*/
								}
#endif
#if 1
								//skeleton
								{
									for(int i = 0; i < srv_zigzag.response.point_skeleton.size(); ++i) {

                                        geometry_msgs::Pose keyPose = srv_zigzag.response.point_skeleton[i];
                                        //ROS_INFO_STREAM("start pose:(" << startPose.position.x << "," << startPose.position.y << ")");
                                        //marker start pose
                                        geometry_msgs::Pose  keySkeletonPose;
                                        keySkeletonPose.position.x = keyPose.position.x;
                                        keySkeletonPose.position.y = keyPose.position.y;
                                        keySkeletonPose.orientation.w = 1.0;

                                        geometry_msgs::Vector3 keySkeletonScale;
                                        keySkeletonScale.x = 0.1;
                                        keySkeletonScale.y = 0.1;
                                        keySkeletonScale.z = 0.1;

                                        std_msgs::ColorRGBA keySkeletonColor;
                                        keySkeletonColor.a = 1.0;
                                        keySkeletonColor.b = 1.0;


                                        visualization_msgs::Marker markerSphere = createMarker("KeyStraightSkeleton",
                                                                        visualization_msgs::Marker::SPHERE,
                                                                        keySkeletonPose,
                                                                        keySkeletonScale,
                                                                        keySkeletonColor,
                                                                        plannerStartId,
                                                                        srv_zigzag.request.map.header.frame_id);
                                        plannerStartId++;

                                        markerArray.markers.push_back(markerSphere);
									}
									/*if(true)
									{
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

										int32_t markerArrowId = srv_zigzag.response.point_skeleton.size();

										visualization_msgs::Marker markerArrow = createMarker("markerArrow",
																			visualization_msgs::Marker::ARROW,
																			markerArrowPose,
																			markerArrowScale,
																			markerArrowColor,
																			plannerStartId,
																			srv_zigzag.request.map.header.frame_id);

											//arrowHead, arrowEnd
										geometry_msgs::Point p;
										p.x = startPose.position.x;
										p.y = startPose.position.y;
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
									}*/
								}
#endif

#if 1 
								//coverage_path
								{
                                    //srv_zigzag.response.path.size()
								for(int index = 0;index < srv_zigzag.response.path.size();index++){
									geometry_msgs::PoseStamped startPose = srv_zigzag.response.path[index].poses[0];
									//marker start pose
									geometry_msgs::Pose  plannerStartPose;
									plannerStartPose.position.x = startPose.pose.position.x;
									plannerStartPose.position.y = startPose.pose.position.y;
									plannerStartPose.orientation.w = 1.0;

									geometry_msgs::Vector3 plannerStartScale;
									plannerStartScale.x = 0.1;
									plannerStartScale.y = 0.1;
									plannerStartScale.z = 0.1;

									std_msgs::ColorRGBA plannerStartColor;
									plannerStartColor.a = 1.0;
									plannerStartColor.r = 1.0;


									visualization_msgs::Marker markerSphereStart = createMarker("PlannerStart",
																	visualization_msgs::Marker::SPHERE,
																	plannerStartPose,
																	plannerStartScale,
																	plannerStartColor,
																	plannerStartId,
																	srv_zigzag.request.map.header.frame_id);
									plannerStartId++;
									markerArray.markers.push_back(markerSphereStart);

									geometry_msgs::Point last_point;
									last_point.x = startPose.pose.position.x;
									last_point.y = startPose.pose.position.y;

									for(int i = 1; i < srv_zigzag.response.path[index].poses.size(); ++i) {

											geometry_msgs::PoseStamped pose = srv_zigzag.response.path[index].poses[i];
											
											geometry_msgs::Pose  markerArrowPose;
											markerArrowPose.position.x = last_point.x;
											markerArrowPose.position.y = last_point.y;
											markerArrowPose.position.z = 0;
											markerArrowPose.orientation.w = 1.0;

											geometry_msgs::Vector3 markerArrowScale;
											markerArrowScale.x = 0.05;
											markerArrowScale.y = 0.05;
											markerArrowScale.z = 0.05;

											std_msgs::ColorRGBA markerArrowColor;
											markerArrowColor.a = 1.0;
											markerArrowColor.r = 1.0;

											visualization_msgs::Marker markerArrow = createMarker("markerArrow",
																			visualization_msgs::Marker::ARROW,
																			markerArrowPose,
																			markerArrowScale,
																			markerArrowColor,
																			plannerStartId,
																			srv_zigzag.request.map.header.frame_id);
											plannerStartId++;
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
									}
								}

#endif 
									marker_pub.publish(markerArray);
									ros::spinOnce();
									loop_rate.sleep();
							}
					}
					else
							ROS_ERROR("Failed to call service /sweeper/zigzag_cpp");
			} else {
							ROS_ERROR("Failed to call service /sweeper/map_modify_srv");
			}
	}
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "scale_map_client");
	
	ros::NodeHandle n;
	
	cpp_test dt(n);
	ros::spin();

	return 0;
}

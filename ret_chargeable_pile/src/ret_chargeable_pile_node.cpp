#include <ros/ros.h>
#include <ros/topic.h>
#include <std_msgs/Float32.h>
#include <actionlib/server/simple_action_server.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <dynamic_reconfigure/server.h>


#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <nav_msgs/Odometry.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <kobuki_msgs/BumperEvent.h>

#include <opencv2/core/core.hpp>

#include <string>
#include <iostream>
#include <algorithm> 

#include <ar_track_alvar_msgs/AlvarMarker.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <ret_chargeable_pile/CheckBumper.h>


#define PI 3.1415926

using namespace tf;
using namespace std;

#define N_MP 5
tf::TransformListener *tf_listener;
double max_frequency;

std::string camera_frame;
std::string world_frame;
std::string string_frame_pose = "_2_pose";
std::string string_frame = "_2";
double half_base_line = 0.06;

bool marker_is_visible = false;
bool is_forward_marker = false;
bool is_position_unsuitable = false;
bool is_angle_unsuitable = false;
bool forward_to_marker_nearby = false;
bool robot_reach_goal = false;
bool start_alter_robot = false;

typedef struct mp{
	double x;
	double y;
	double z;
	double p;

	mp(double x_,double y_,double z_,double p_){x = x_;y = y_;z = z_;p = p_;}
}marker_pose;

std::vector<marker_pose> temp_marker_pose;
std::vector<marker_pose> positive_vec;
std::vector<marker_pose> negative_vec;

geometry_msgs::Twist rotate_move_cmd;
geometry_msgs::Twist straight_move_cmd;
geometry_msgs::Twist stop_move_cmd;
geometry_msgs::Twist common_move_cmd;

double marker_x;
double marker_y;
double marker_z;
double alter_marker_x;
double alter_marker_y;
double alter_marker_z;

double robot_theta;
double robot_x;
double robot_y;

double marker_yaw;
double marker_roll;
double marker_pitch;

double position_threshold = 0.09;
double angle_threshold = 0.09;
double angular_tolerance = 2.0*PI/180; 
double rate_frequency = 100;

double wz = 0.6;
double vx = 0.12;

double robot_to_marker_distance = 0.6;

string odom_frame = "/odom";
string base_frame = "/base_link";

class robot_control{
	public:
	ros::NodeHandle nh;
	ros::Subscriber msg_sub;
	ros::Subscriber bumper_sub;
	ros::Subscriber odom_sub;
	ros::Publisher cmd_vel_pub;

    boost::mutex mtx;
    boost::mutex mtx1;
    boost::mutex mtx2;
    boost::mutex mtx3;
    boost::mutex mtx4;
    boost::mutex mtx5;

	geometry_msgs::Twist move_cmd;

    ros::ServiceClient check_bumper_client;

	robot_control(ros::NodeHandle n):nh(n)
	{
		msg_sub = nh.subscribe("/ar_pose_marker", 1000,&robot_control::poseCallback,this);
        odom_sub = nh.subscribe("/odom",1000,&robot_control::odomCallback,this);
		cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity",1000);

        check_bumper_client = nh.serviceClient<ret_chargeable_pile::CheckBumper>("/sweeper/CheckBumper");
	}

	void poseCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr &msg);
	void robot_move_base();
	void rotate_n_angle(double n,int direction);
	void straight_n_distance(double n,bool check_bumper);
    void odomCallback(const nav_msgs::Odometry& msg);
    double angle_wrap(double angle);
};

    double robot_control::angle_wrap(double angle){
        double angle_res;
        if(angle > 0){
            double angle_temp = angle/(2*PI);
            int n_p = std::floor(angle_temp);
            angle_res = angle-n_p*(2*PI);
            if(angle_res > PI)
                angle_res -= (2*PI);
        }
        else{
            double angle_temp = angle/(2*PI);
            int n_p = std::ceil(angle_temp);
            angle_res = angle-n_p*(2*PI);
            if(fabs(angle_res) > PI)
                angle_res += (2*PI);

        }

        return angle_res;
    }
    void robot_control::odomCallback(const nav_msgs::Odometry& msg){
        mtx.lock();
       robot_x = msg.pose.pose.position.x;
       robot_y = msg.pose.pose.position.y;
       tf::Quaternion q(msg.pose.pose.orientation.x,\
                        msg.pose.pose.orientation.y,\
                        msg.pose.pose.orientation.z,\
                        msg.pose.pose.orientation.w);

       double roll,pitch,yaw;
       tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
       robot_theta = angle_wrap(yaw);

	if(!marker_is_visible)
		cmd_vel_pub.publish(rotate_move_cmd);
       mtx.unlock();
    }
	void robot_control::rotate_n_angle(double n,int direction)
	{
        mtx2.lock();
		if(direction  < 0)
		{	
			common_move_cmd.linear.x = 0;
			common_move_cmd.linear.y = 0;
			common_move_cmd.linear.z = 0;
			common_move_cmd.angular.x = 0;
			common_move_cmd.angular.y = 0;
			common_move_cmd.angular.z = -wz;
		}
		else
		{
			common_move_cmd.linear.x = 0;
			common_move_cmd.linear.y = 0;
			common_move_cmd.linear.z = 0;
			common_move_cmd.angular.x = 0;
			common_move_cmd.angular.y = 0;
			common_move_cmd.angular.z = wz;
		}

		tf::StampedTransform OdomToBaselink;
		try{
            tf_listener->waitForTransform(odom_frame, base_frame, ros::Time(0), ros::Duration(2.0) );
		    tf_listener->lookupTransform(odom_frame, base_frame, ros::Time(0), OdomToBaselink);
        }
        catch (tf::TransformException ex){
          ROS_INFO("Cannot find transform between /odom and /base_link");
          cmd_vel_pub.publish(geometry_msgs::Twist());
          ros::shutdown();
        }

		double last_angle = fabs(tf::getYaw(OdomToBaselink.getRotation()));
   	    double turn_angle = 0;
		ros::Rate loopRate(rate_frequency);
        while( (fabs(turn_angle + angular_tolerance) < n) && (ros::ok()) ){
            cmd_vel_pub.publish(common_move_cmd);
            loopRate.sleep();
            tf_listener->lookupTransform(odom_frame, base_frame, ros::Time(0), OdomToBaselink);
     
            double rotation = fabs(tf::getYaw(OdomToBaselink.getRotation()));
            double delta_angle = fabs(rotation - last_angle);

            turn_angle += delta_angle;
            last_angle = rotation;
        }
		cmd_vel_pub.publish(geometry_msgs::Twist());
        mtx2.unlock();
	}

	void robot_control::straight_n_distance(double n,bool check_bumper)
	{
        mtx3.lock();
		common_move_cmd.linear.x = vx;
		common_move_cmd.linear.y = 0;
		common_move_cmd.linear.z = 0;
		common_move_cmd.angular.x = 0;
		common_move_cmd.angular.y = 0;
		common_move_cmd.angular.z = 0;
    
        if(check_bumper){
            ROS_INFO("check bumper and stop...");
            ros::Rate lr(100);
            bool res_srv = false;
            ret_chargeable_pile::CheckBumper cb_srv;
            cb_srv.request.check = true;

            while(ros::ok()){
                cmd_vel_pub.publish(common_move_cmd);
                lr.sleep();
                
                res_srv = check_bumper_client.call(cb_srv);
                if(res_srv){
                    if(cb_srv.response.active){
                        cmd_vel_pub.publish(geometry_msgs::Twist());
                        robot_reach_goal = true;
                        mtx3.unlock();
                        return;
                    }
                }
            }
        }
		tf::StampedTransform OdomToBaselink;
		try{
            tf_listener->waitForTransform(odom_frame, base_frame, ros::Time(0), ros::Duration(2.0) );
		    tf_listener->lookupTransform(odom_frame, base_frame, ros::Time(0), OdomToBaselink);
        }
        catch (tf::TransformException ex){
          ROS_INFO("Cannot find transform between /odom and /base_link");
          cmd_vel_pub.publish(geometry_msgs::Twist());
          ros::shutdown();
        }

		ros::Rate loopRate(rate_frequency);

		float x_start = OdomToBaselink.getOrigin().x();
        float y_start = OdomToBaselink.getOrigin().y();

        float distance = 0;
        while( (distance < n) && (ros::ok())){
            cmd_vel_pub.publish(common_move_cmd);
            loopRate.sleep();
            tf_listener->lookupTransform(odom_frame, base_frame, ros::Time(0), OdomToBaselink);
            float x = OdomToBaselink.getOrigin().x();
            float y = OdomToBaselink.getOrigin().y();

		    ROS_INFO("x:%f,y:%f",x,y);

            distance = sqrt(pow((x - x_start), 2) +  pow((y - y_start), 2));
        }
        cmd_vel_pub.publish(geometry_msgs::Twist());
        mtx3.unlock();
	}

	void robot_control::poseCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr &msg)
	{
        mtx4.lock();
		ROS_INFO("world_frame:%s\n",world_frame.c_str());
		static int count_msg = 0;

#if 1

		if(msg->header.frame_id == world_frame)
		{
			ROS_INFO("world_frame:%s\n",world_frame.c_str());
#if 1
			ROS_INFO("markers.size:%d",(size_t)msg->markers.size());
			if(msg->markers.size() <= 0){
				is_forward_marker = false;
				is_position_unsuitable = false;
				is_angle_unsuitable = false;
				forward_to_marker_nearby = false;
				//cmd_vel_pub.publish(rotate_move_cmd);

				mtx4.unlock();
				return;
			}
				
			ar_track_alvar_msgs::AlvarMarker marker = msg->markers[0];
			if(marker.header.frame_id == world_frame+string_frame_pose)
			{
				ROS_INFO("world_frame:%s\n",(world_frame+string_frame_pose).c_str());
#if 1
				if(marker.pose.header.frame_id == world_frame+string_frame)
				{
					
					is_forward_marker = false;
					is_position_unsuitable = false;
					is_angle_unsuitable = false;
					forward_to_marker_nearby = false;

					ROS_INFO("world_frame:%s\n",(world_frame+string_frame).c_str());
					marker_is_visible = true;
#if 1

					tf::StampedTransform CameraToWorld;
    			    try{
						//tf_listener->waitForTransform(world_frame+"_2", camera_frame, ros::Time(0), ros::Duration(2.0));
						//tf_listener->lookupTransform(world_frame+"_2", camera_frame, ros::Time(0), CameraToWorld);

						tf_listener->waitForTransform(camera_frame,world_frame+string_frame, ros::Time(0), ros::Duration(2.0));
						tf_listener->lookupTransform(camera_frame,world_frame+string_frame, ros::Time(0), CameraToWorld);	
   				    }
    			    catch (tf::TransformException ex){
					    ROS_ERROR("%s",ex.what());
						cmd_vel_pub.publish(geometry_msgs::Twist());
						ros::shutdown();
    			    }

					tf::Transform CToW = CameraToWorld.inverse();

					marker_x = CToW.getOrigin().x();
					marker_y = CToW.getOrigin().y();
					marker_z = CToW.getOrigin().z();

					CToW.getBasis().getRPY(marker_roll,marker_pitch,marker_yaw);

					alter_marker_x = marker_x;
					alter_marker_y = marker_y - half_base_line*cos(marker_pitch);
					alter_marker_z = marker_z - half_base_line*sin(marker_pitch);

					ROS_INFO("alter_marker_x:%f\n",alter_marker_x);
					ROS_INFO("alter_marker_y:%f\n",alter_marker_y);
					ROS_INFO("alter_marker_z:%f\n",alter_marker_z);
					ROS_INFO("marker_pitch:%f\n",marker_pitch);
					
					if(start_alter_robot){
        				mtx4.unlock();
						return;
					}

					if(temp_marker_pose.size() < N_MP){
						temp_marker_pose.push_back(marker_pose(alter_marker_x,alter_marker_y,alter_marker_z,marker_pitch));
						start_alter_robot = false;
						ROS_INFO("store marker pose to %d",N_MP);
						mtx4.unlock();
						return;
					}
					else{
						ROS_INFO("start alter robot...");
						start_alter_robot = true;
						
						int positive_count = 0;
						int negative_count = 0;
						for(int i = 0;i < temp_marker_pose.size();i++){
							if(temp_marker_pose[i].y >= 0.0f){
								positive_count++;
								positive_vec.push_back(temp_marker_pose[i]);
							}
							else{
								negative_count++;
								negative_vec.push_back(temp_marker_pose[i]);
							}
						}
						if(positive_count > negative_count){
							double positive_sum_x = 0;
							double positive_sum_y = 0;
							double positive_sum_z = 0;
							double positive_sum_p = 0;

							for(int j = 0;j < positive_vec.size();j++){
								positive_sum_x += positive_vec[j].x;
								positive_sum_y += positive_vec[j].y;
								positive_sum_z += positive_vec[j].z;
								positive_sum_p += positive_vec[j].p;
							}

							alter_marker_x = positive_sum_x/positive_vec.size();
							alter_marker_y = positive_sum_y/positive_vec.size();
							alter_marker_z = positive_sum_z/positive_vec.size();
							marker_pitch = positive_sum_p/positive_vec.size();

						}
						else{
							double negative_sum_x = 0;
							double negative_sum_y = 0;
							double negative_sum_z = 0;
							double negative_sum_p = 0;

							for(int j = 0;j < negative_vec.size();j++){
								negative_sum_x += negative_vec[j].x;
								negative_sum_y += negative_vec[j].y;
								negative_sum_z += negative_vec[j].z;
								negative_sum_p += negative_vec[j].p;
							}

							alter_marker_x = negative_sum_x/negative_vec.size();
							alter_marker_y = negative_sum_y/negative_vec.size();
							alter_marker_z = negative_sum_z/negative_vec.size();
							marker_pitch = negative_sum_p/negative_vec.size();
						}

						ROS_INFO("positive_count:%d",positive_count);
						ROS_INFO("negative_count:%d",negative_count);
					}
					
					ROS_INFO("re_alter_marker_x:%f\n",alter_marker_x);
					ROS_INFO("re_alter_marker_y:%f\n",alter_marker_y);
					ROS_INFO("re_alter_marker_z:%f\n",alter_marker_z);
					ROS_INFO("marker_pitch:%f\n",marker_pitch);

					if(fabs(alter_marker_y) < position_threshold)
					{
						if(fabs(marker_pitch) < angle_threshold)
						{
							is_forward_marker = true;
							is_angle_unsuitable = false;
							is_position_unsuitable = false;
						}
						else
						{
							is_forward_marker = false;
							is_position_unsuitable = false;
							is_angle_unsuitable = true;
						}

					}
					else
					{
						if(fabs(marker_pitch) < angle_threshold)
						{
							is_forward_marker = false;
							is_position_unsuitable = true;
							is_angle_unsuitable = false;
						}
						else
						{
							is_forward_marker = false;
							is_position_unsuitable = true;			
							is_angle_unsuitable = true;
						}
					}

					if(alter_marker_z > robot_to_marker_distance){
						forward_to_marker_nearby = true;
						is_position_unsuitable = true;
					}

					std::vector<marker_pose>().swap(temp_marker_pose);
					std::vector<marker_pose>().swap(positive_vec);
					std::vector<marker_pose>().swap(negative_vec);

					if(is_forward_marker){
			        	ROS_INFO("Is_Forward_Marker and Start Straight Line Move");
				    	ros::Duration(1.0).sleep();
			        	straight_n_distance(0,true);
			    	}
			    	else{
				    	ROS_INFO("Not Is_Forward_Marker and Alter Robot Position and Direction");
				    	ros::Duration(0.5).sleep();		
				    	robot_move_base();				
		        	}
#endif
			}
			else{
				marker_is_visible = false;
				//cmd_vel_pub.publish(rotate_move_cmd);
			}
#endif
		}
		else{
			marker_is_visible = false;
			//cmd_vel_pub.publish(rotate_move_cmd);
		}
#endif
		}
		else{
			marker_is_visible = false;
			//cmd_vel_pub.publish(rotate_move_cmd);
		}
#endif
        mtx4.unlock();
	}
void robot_control::robot_move_base()
{
		if(!start_alter_robot)
			return;

        mtx5.lock();
		ROS_INFO("is_position_unsuitable:%d",is_position_unsuitable);
        ROS_INFO("is_angle_unsuitable:%d",is_angle_unsuitable);
		ROS_INFO("forward_to_marker_nearby:%d",forward_to_marker_nearby);
#if 1
			if(is_position_unsuitable)
			{
				if(forward_to_marker_nearby)
				{
					if(alter_marker_y > position_threshold)
					{
						//double angle_to_nearby = cv::fastAtan2(alter_marker_z-robot_to_marker_distance,alter_marker_y);
						double angle_to_nearby = atan((alter_marker_z-robot_to_marker_distance)/alter_marker_y);
                        
						ROS_INFO("angle_to_nearby:%f",angle_to_nearby);
						ROS_INFO("robot_theta:%f",robot_theta);
                        
                        double rotation_to_nearby = PI/2+marker_pitch-angle_to_nearby;
                        double rotation_to_nearby_back = PI/2-angle_to_nearby;
						
                        ros::Duration(0.5).sleep();
                        rotate_n_angle(rotation_to_nearby,-1);
                        ros::Duration(0.5).sleep();

                        double distance_to_nearby = \
                                    sqrt(pow((alter_marker_z - robot_to_marker_distance), 2) +  pow((alter_marker_y), 2));
 
                        straight_n_distance(distance_to_nearby,false);
                        ros::Duration(0.5).sleep();

                        rotate_n_angle(rotation_to_nearby_back,1);
                        ros::Duration(0.5).sleep();
					}
					else if(alter_marker_y < -position_threshold)
					{
                        //double angle_to_nearby = cv::fastAtan2(alter_marker_z-robot_to_marker_distance,alter_marker_y);
                        double angle_to_nearby = atan((alter_marker_z-robot_to_marker_distance)/alter_marker_y);
                        
						ROS_INFO("angle_to_nearby:%f",angle_to_nearby);
						ROS_INFO("robot_theta:%f",robot_theta);
                        
                        double rotation_to_nearby = PI/2-marker_pitch+angle_to_nearby;
                        double rotation_to_nearby_back = PI/2+angle_to_nearby;
						
                        ros::Duration(0.5).sleep();
                        rotate_n_angle(rotation_to_nearby,1);
                        ros::Duration(0.5).sleep();

                        double distance_to_nearby = \
                                    sqrt(pow((alter_marker_z - robot_to_marker_distance), 2) +  pow((alter_marker_y), 2));
 
                        straight_n_distance(distance_to_nearby,false);
                        ros::Duration(0.5).sleep();

                        rotate_n_angle(rotation_to_nearby_back,-1);
                        ros::Duration(0.5).sleep();
					}
				}
				else
				{
					if(alter_marker_y > position_threshold)
					{
                        
                        {
						    ros::Duration(0.5).sleep();
                            rotate_n_angle(PI/2+marker_pitch,-1);        
						    ros::Duration(0.5).sleep();
						    straight_n_distance(fabs(alter_marker_y/1),false);
						    ros::Duration(0.5).sleep();
                            rotate_n_angle(PI/2,1);        
						    ros::Duration(0.5).sleep();
                        }
					}
					else if(alter_marker_y < -position_threshold)
					{
                        {
						    ros::Duration(0.5).sleep();
                            rotate_n_angle(PI/2-marker_pitch,1);        
						    ros::Duration(0.5).sleep();
						    straight_n_distance(fabs(alter_marker_y/1),false);
						    ros::Duration(0.5).sleep();
                            rotate_n_angle(PI/2,-1);        
						    ros::Duration(0.5).sleep();
                        }
					}
                    else{
                         if(marker_pitch < 0){
						    ros::Duration(0.5).sleep();
                            rotate_n_angle(fabs(marker_pitch),-1);        
						    ros::Duration(0.5).sleep();
                         }
                        else{
						    ros::Duration(0.5).sleep();
                            rotate_n_angle(fabs(marker_pitch),1);        
						    ros::Duration(0.5).sleep();
                         }
                    }
				}
			}
			else{
                if(is_angle_unsuitable){
				    if(marker_pitch < 0)
				    {
						ros::Duration(0.5).sleep();
						rotate_n_angle(fabs(marker_pitch),-1);
						ros::Duration(0.5).sleep();
					}
					else
					{
						ros::Duration(0.5).sleep();
						rotate_n_angle(fabs(marker_pitch),1);
						ros::Duration(0.5).sleep();
					}
                }
			}		

#endif	
	start_alter_robot = false;
        mtx5.unlock();
	}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ret_chargeable_pile");
	ros::NodeHandle n,pn("~");

	robot_control rc(n);

	pn.param<string>("camera_frame", camera_frame, "mynteye_left_frame");
    pn.param<string>("world_frame", world_frame, "ar_marker");
	pn.param<double>("max_freq", max_frequency, 10.0);

	rotate_move_cmd.linear.x = 0;
	rotate_move_cmd.linear.y = 0;
	rotate_move_cmd.linear.z = 0;
	rotate_move_cmd.angular.x = 0;
	rotate_move_cmd.angular.y = 0;
	rotate_move_cmd.angular.z = -0.8;

	straight_move_cmd.linear.x = 0.2;
	straight_move_cmd.linear.y = 0;
	straight_move_cmd.linear.z = 0;
	straight_move_cmd.angular.x = 0;
	straight_move_cmd.angular.y = 0;
	straight_move_cmd.angular.z = 0;

	stop_move_cmd.linear.x = 0;
	stop_move_cmd.linear.y = 0;
	stop_move_cmd.linear.z = 0;
	stop_move_cmd.angular.x = 0;
	stop_move_cmd.angular.y = 0;
	stop_move_cmd.angular.z = 0;

	tf_listener = new tf::TransformListener(n);

    ROS_INFO("Start Return To Chargeable Pile");
	ros::Rate rate(max_frequency);
	while(ros::ok()){
		ros::spinOnce();
		rate.sleep();

        if(robot_reach_goal)
        	ros::shutdown();
	}

  return 0;
}

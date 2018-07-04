/* A ROS implementation of the Pure pursuit path tracking algorithm (Coulter 1992).

   Terminology (mostly :) follows:
   Coulter, Implementation of the pure pursuit algoritm, 1992 and 
   Sorniotti et al. Path tracking for Automated Driving, 2017.
 */

#include <string>
#include <cmath>
#include <algorithm>

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <ackermann_msgs/AckermannDriveStamped.h>

#include <kdl/frames.hpp>

// TODO: Figure out how to use tf2 DataConversions
// for more elegant and compact code
//#include <tf2_kdl/tf2_kdl.h>
//#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <dynamic_reconfigure/server.h>
#include <pure_pursuit/myntMoveTrackServiceConfig.h>
#include <pure_pursuit/myntMoveTrack.h>

using std::string;

class myntMoveTrackService
{
public:

  //! Constructor
  myntMoveTrackService();

  //! Compute velocit commands each time new odometry data is received.
  void track(geometry_msgs::PoseStamped nextPoint);

  //! Compute transform that transforms a pose into the robot frame (base_link)
  KDL::Frame transformToBaseLink(const geometry_msgs::Pose& pose,
                                 const geometry_msgs::Transform& tf);

  bool myntMoveTrack(pure_pursuit::myntMoveTrack::Request  &req, pure_pursuit::myntMoveTrack::Response &res);
  
  //! Helper founction for computing eucledian distances in the x-y plane.
  template<typename T1, typename T2>
  double distance(T1 pt1, T2 pt2)
  {
    return sqrt(pow(pt1.x - pt2.x,2) + pow(pt1.y - pt2.y,2) + pow(pt1.z - pt2.z,2));
  }



private:

  //! Dynamic reconfigure callback.
  void reconfigure(pure_pursuit::myntMoveTrackServiceConfig &config, uint32_t level);
  
  // Vehicle parameters
  double L_;
  // Algorithm variables
  // Position tolerace is measured along the x-axis of the robot!
  double ld_, pos_tol_;
  // Generic control variables
  double v_max_, v_, w_max_;
  // Control variables for Ackermann steering
  // Steering angle is denoted by delta
  double delta_, delta_vel_, acc_, jerk_, delta_max_, rate_;
  nav_msgs::Path path_;
  unsigned idx_;
  bool goal_reached_;
  geometry_msgs::Twist cmd_vel_;
  ackermann_msgs::AckermannDriveStamped cmd_acker_;
  
  // Ros infrastructure
  ros::NodeHandle nh_, nh_private_;
  ros::Publisher pub_vel_, pub_acker_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  geometry_msgs::TransformStamped lookahead_;
  string map_frame_id_, robot_frame_id_, lookahead_frame_id_, acker_frame_id_;

  dynamic_reconfigure::Server<pure_pursuit::myntMoveTrackServiceConfig> reconfigure_server_;
  dynamic_reconfigure::Server<pure_pursuit::myntMoveTrackServiceConfig>::CallbackType reconfigure_callback_;
  
};

myntMoveTrackService::myntMoveTrackService() : ld_(1.0), v_max_(0.1), v_(v_max_), w_max_(1.0), pos_tol_(0.1), idx_(0),
                             goal_reached_(true), nh_private_("~"), tf_listener_(tf_buffer_),
                             map_frame_id_("map"), robot_frame_id_("base_link"),
                             lookahead_frame_id_("lookahead")
{
  // Get parameters from the parameter server
  nh_private_.param<double>("wheelbase", L_, 1.0);
  nh_private_.param<double>("lookahead_distance", ld_, 0.5);
  //nh_private_.param<double>("linear_velocity", v_, 0.1);
  nh_private_.param<double>("max_rotational_velocity", w_max_, 1.0);
  nh_private_.param<double>("position_tolerance", pos_tol_, 0.05);
  nh_private_.param<double>("steering_angle_velocity", delta_vel_, 100.0);
  nh_private_.param<double>("acceleration", acc_, 100.0);
  nh_private_.param<double>("jerk", jerk_, 100.0);
  nh_private_.param<double>("steering_angle_limit", delta_max_, 1.57);
  nh_private_.param<double>("rate", rate_, 10.0);
  nh_private_.param<string>("map_frame_id", map_frame_id_, "map");
  // Frame attached to midpoint of rear axle (for front-steered vehicles).
  nh_private_.param<string>("robot_frame_id", robot_frame_id_, "base_footprint");
  // Lookahead frame moving along the path as the vehicle is moving.
  nh_private_.param<string>("lookahead_frame_id", lookahead_frame_id_, "lookahead");
  // Frame attached to midpoint of front axle (for front-steered vehicles).
  nh_private_.param<string>("ackermann_frame_id", acker_frame_id_, "rear_axle_midpoint");

  // Populate messages with static data
  lookahead_.header.frame_id = robot_frame_id_;
  lookahead_.child_frame_id = lookahead_frame_id_;

  cmd_acker_.header.frame_id = acker_frame_id_;
  cmd_acker_.drive.steering_angle_velocity = delta_vel_;
  cmd_acker_.drive.acceleration = acc_;
  cmd_acker_.drive.jerk = jerk_;
  
  pub_vel_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1);
  pub_acker_ = nh_.advertise<ackermann_msgs::AckermannDriveStamped>("cmd_acker", 1);

  reconfigure_callback_ = boost::bind(&myntMoveTrackService::reconfigure, this, _1, _2);
  reconfigure_server_.setCallback(reconfigure_callback_);
}

void myntMoveTrackService::track(geometry_msgs::PoseStamped nextPoint)
{

  goal_reached_ = false;

  ros::Rate loop_rate(rate_);

  while(!goal_reached_)
  {

	  // The velocity commands are computed, each time a new Odometry message is received.
	  // Odometry is not used directly, but through the currentPoseTf tree.

	  // Get the current robot pose
	  geometry_msgs::TransformStamped currentPoseTf;
	  try
	  {
		  currentPoseTf = tf_buffer_.lookupTransform(map_frame_id_, robot_frame_id_, ros::Time(0));

		  // Transformed lookahead to base_link frame is lateral error
		  KDL::Frame F_bl_ld = transformToBaseLink(nextPoint.pose, currentPoseTf.transform);   //diff with currentPos and nextPoint.pose
		  lookahead_.transform.translation.x = F_bl_ld.p.x();
		  lookahead_.transform.translation.y = F_bl_ld.p.y();
		  lookahead_.transform.translation.z = F_bl_ld.p.z();
		  F_bl_ld.M.GetQuaternion(lookahead_.transform.rotation.x,
								  lookahead_.transform.rotation.y,
								  lookahead_.transform.rotation.z,
								  lookahead_.transform.rotation.w);

		  KDL::Frame F_bl_end = F_bl_ld;

		  if (fabs(F_bl_end.p.x()) <= pos_tol_)
		  {
			  // We have reached the goal
			  goal_reached_ = true;

		  }else{

			  // We need to extend the lookahead distance
			  // beyond the goal point.

			  // Find the intersection between the circle of radius ld
			  // centered at the robot (origin)
			  // and the line defined by the last path pose
			  double roll, pitch, yaw;
			  F_bl_end.M.GetRPY(roll, pitch, yaw);
			  double k_end = tan(yaw); // Slope of line defined by the last path pose
			  double l_end = F_bl_end.p.y() - k_end * F_bl_end.p.x();
			  double a = 1 + k_end * k_end;
			  double b = 2 * l_end;
			  double c = l_end * l_end - ld_ * ld_;
			  double D = sqrt(b*b - 4*a*c);
			  double x_ld = (-b + copysign(D,v_)) / (2*a);
			  double y_ld = k_end * x_ld + l_end;

			  lookahead_.transform.translation.x = x_ld;
			  lookahead_.transform.translation.y = y_ld;
			  lookahead_.transform.translation.z = F_bl_end.p.z();
			  F_bl_end.M.GetQuaternion(lookahead_.transform.rotation.x,
									   lookahead_.transform.rotation.y,
									   lookahead_.transform.rotation.z,
									   lookahead_.transform.rotation.w);

		  }


		  if (!goal_reached_)
		  {
			  // We are tracking.

			  // Compute linear velocity.
			  // Right now,this is not very smart :)
			  v_ = copysign(v_max_, v_);

			  // Compute the angular velocity.
			  // Lateral error is the y-value of the lookahead point (in base_link frame)
			  double yt = lookahead_.transform.translation.y;
			  double ld_2 = ld_ * ld_;
			  cmd_vel_.angular.z = std::min( 2*v_ / ld_2 * yt, w_max_ );

			  // Compute desired Ackermann steering angle
			  cmd_acker_.drive.steering_angle = std::min( atan2(2 * yt * L_, ld_2), delta_max_ );

			  // Set linear velocity for tracking.
			  cmd_vel_.linear.x = v_;
			  cmd_acker_.drive.speed = v_;

			  cmd_acker_.header.stamp = ros::Time::now();
		  }
		  else
		  {
			  // We are at the goal!

			  // Stop the vehicle

			  // The lookahead target is at our current pose.
			  lookahead_.transform = geometry_msgs::Transform();
			  lookahead_.transform.rotation.w = 1.0;

			  // Stop moving.
			  cmd_vel_.linear.x = 0.0;
			  cmd_vel_.angular.z = 0.0;

			  cmd_acker_.header.stamp = ros::Time::now();
			  cmd_acker_.drive.steering_angle = 0.0;
			  cmd_acker_.drive.speed = 0.0;
		  }

		  // Publish the lookahead target transform.
		  lookahead_.header.stamp = ros::Time::now();
		  tf_broadcaster_.sendTransform(lookahead_);

		  // Publish the velocities
		  pub_vel_.publish(cmd_vel_);

		  // Publish ackerman steering setpoints
		  pub_acker_.publish(cmd_acker_);
	  }
	  catch (tf2::TransformException &ex)
	  {
		  ROS_WARN_STREAM(ex.what());
	  }

      loop_rate.sleep();

  }


}

KDL::Frame myntMoveTrackService::transformToBaseLink(const geometry_msgs::Pose& pose,
                                            const geometry_msgs::Transform& tf)
{
  // Pose in global (map) frame
  KDL::Frame F_map_pose(KDL::Rotation::Quaternion(pose.orientation.x,
                                                  pose.orientation.y,
                                                  pose.orientation.z,
                                                  pose.orientation.w),
                        KDL::Vector(pose.position.x,
                                    pose.position.y,
                                    pose.position.z));

  // Robot (base_link) in global (map) frame
  KDL::Frame F_map_tf(KDL::Rotation::Quaternion(tf.rotation.x,
                                                tf.rotation.y,
                                                tf.rotation.z,
                                                tf.rotation.w),
                      KDL::Vector(tf.translation.x,
                                  tf.translation.y,
                                  tf.translation.z));

  // TODO: See how the above conversions can be done more elegantly
  // using tf2_kdl and tf2_geometry_msgs

  return F_map_tf.Inverse()*F_map_pose;
}


bool myntMoveTrackService::myntMoveTrack(pure_pursuit::myntMoveTrack::Request  &req, pure_pursuit::myntMoveTrack::Response &res)
{

  geometry_msgs::PoseStamped nextPoint = req.goal;

  if (nextPoint.header.frame_id == map_frame_id_)
  {

	  myntMoveTrackService::track(nextPoint);

	  res.err_no = 0;
	  res.err_msg = "success";

  }
  else
  {
    ROS_WARN_STREAM("The path must be published in the " << map_frame_id_
                    << " frame! Ignoring path in " << nextPoint.header.frame_id
                    << " frame!");
	res.err_no = 1;
	res.err_msg = "bad frame_id";
  }
  
  return true;
}


void myntMoveTrackService::reconfigure(pure_pursuit::myntMoveTrackServiceConfig &config, uint32_t level)
{
  v_max_ = config.max_linear_velocity;
}

int main(int argc, char**argv)
{
  ros::init(argc, argv, "mynt_move_track_service");
  ros::NodeHandle n;


  myntMoveTrackService trackServiceObject ;
  ros::ServiceServer service = n.advertiseService("mynt_move_track", &myntMoveTrackService::myntMoveTrack,&trackServiceObject);
  ROS_INFO("Ready to mynt_move_track.");
  ros::spin();


  return 0;
}

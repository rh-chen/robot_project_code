    /* A ROS implementation of the Pure pursuit path tracking algorithm (Coulter 1992).

   Terminology (mostly :) follows:
   Coulter, Implementation of the pure pursuit algoritm, 1992 and 
   Sorniotti et al. Path tracking for Automated Driving, 2017.
 */

#include <string>
#include <cmath>
#include <algorithm>

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <ackermann_msgs/AckermannDriveStamped.h>

#include <kdl/frames.hpp>

#include <dynamic_reconfigure/server.h>
#include <pure_pursuit/PurePursuitConfig.h>

using std::string;

class PurePursuit
{
public:

  //! Constructor
  PurePursuit();
  //! Compute velocit commands each time new odometry data is received.
  void computeVelocities(nav_msgs::Odometry odom);
  //! Receive path to follow.
  void receivePath(nav_msgs::Path path);

  //! Compute transform that transforms a pose into the robot frame (base_link)
  geometry_msgs::Twist diff2D(const tf::Pose& pose1, const tf::Pose& pose2);
  double headingDiff(double x, double y, double pt_x, double pt_y, double heading);
  
  geometry_msgs::Twist limitTwist(const geometry_msgs::Twist& twist);
  bool getRobotPose(tf::Stamped<tf::Pose>& global_pose);
  /*template<typename T1, typename T2>
  double distance(T1 pt1, T2 pt2)
  {
    return sqrt(pow(pt1.x - pt2.x,2) + pow(pt1.y - pt2.y,2) + pow(pt1.z - pt2.z,2));
  }*/
  
//private:

  inline double sign(double n){
    return n < 0.0 ? -1.0 : 1.0;
  }
  //! Dynamic reconfigure callback.
  void reconfigure(pure_pursuit::PurePursuitConfig &config, uint32_t level);

  nav_msgs::Path path_;

  geometry_msgs::Twist cmd_vel_;
  double K_trans_, K_rot_, tolerance_trans_, tolerance_rot_;
  double tolerance_timeout_;
  double max_vel_lin_, max_vel_th_;
  double min_vel_lin_, min_vel_th_;
  double min_in_place_vel_th_, in_place_trans_vel_;
  bool holonomic_;
  std::string p_robot_base_frame_;
  std::string p_global_frame_;

  boost::mutex odom_lock_;
  ros::Subscriber odom_sub_;
  double trans_stopped_velocity_, rot_stopped_velocity_;
  ros::Time goal_reached_time_;
  unsigned int current_waypoint_; 
  std::vector<geometry_msgs::PoseStamped> global_plan_;
  int samples_;

  
  // Ros infrastructure
  ros::NodeHandle nh_, node_private;
  ros::Subscriber sub_odom_, sub_path_;
  ros::Publisher pub_vel_;	

  tf::TransformListener* tf_;

  void initialize(tf::TransformListener* tf);
};

PurePursuit::PurePursuit() : tf_(NULL),node_private("~"),holonomic_(false){}

void PurePursuit::initialize(tf::TransformListener* tf){
  tf_ = tf;
  current_waypoint_ = 0;
  goal_reached_time_ = ros::Time::now();

  node_private.param("k_trans", K_trans_, 2.0);
  node_private.param("k_rot", K_rot_, 2.0);

  node_private.param("tolerance_trans", tolerance_trans_, 0.03);
  node_private.param("tolerance_rot", tolerance_rot_, 0.05);
  node_private.param("tolerance_timeout", tolerance_timeout_, 0.5);

  node_private.param("holonomic", holonomic_, false);

  node_private.param("samples", samples_, 10);

  node_private.param("max_vel_lin", max_vel_lin_, 0.1);
  node_private.param("max_vel_th", max_vel_th_, 0.6);

  node_private.param("min_vel_lin", min_vel_lin_, 0.03);
  node_private.param("min_vel_th", min_vel_th_, 0.0);
  node_private.param("min_in_place_vel_th", min_in_place_vel_th_, 0.0);
  node_private.param("in_place_trans_vel", in_place_trans_vel_, 0.0);

  node_private.param("trans_stopped_velocity", trans_stopped_velocity_, 1e-4);
  node_private.param("rot_stopped_velocity", rot_stopped_velocity_, 1e-4);

  node_private.param("robot_base_frame", p_robot_base_frame_, std::string("base_link"));
  node_private.param("global_frame", p_global_frame_, std::string("map"));

  //ros::NodeHandle node;
  sub_path_ = nh_.subscribe("/trajectory", 1, &PurePursuit::receivePath, this);
  sub_odom_ = nh_.subscribe("/odom", 1, &PurePursuit::computeVelocities, this);
  pub_vel_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1);

  ROS_DEBUG("Initialized");
}
geometry_msgs::Twist PurePursuit::limitTwist(const geometry_msgs::Twist& twist)
{
  geometry_msgs::Twist res = twist;
  res.linear.x *= K_trans_;
  if(!holonomic_)
    res.linear.y = 0.0;
  else    
    res.linear.y *= K_trans_;
  res.angular.z *= K_rot_;

  //make sure to bound things by our velocity limits
  double lin_overshoot = sqrt(res.linear.x * res.linear.x + res.linear.y * res.linear.y) / max_vel_lin_;
  double lin_undershoot = min_vel_lin_ / sqrt(res.linear.x * res.linear.x + res.linear.y * res.linear.y);
  if (lin_overshoot > 1.0) 
  {
    res.linear.x /= lin_overshoot;
    res.linear.y /= lin_overshoot;
  }

  //we only want to enforce a minimum velocity if we're not rotating in place
  if(lin_undershoot > 1.0)
  {
    res.linear.x *= lin_undershoot;
    res.linear.y *= lin_undershoot;
  }

  if (fabs(res.angular.z) > max_vel_th_) res.angular.z = max_vel_th_ * sign(res.angular.z);
  if (fabs(res.angular.z) < min_vel_th_) res.angular.z = min_vel_th_ * sign(res.angular.z);

  //we want to check for whether or not we're desired to rotate in place
  if(sqrt(twist.linear.x * twist.linear.x + twist.linear.y * twist.linear.y) < in_place_trans_vel_){
    if (fabs(res.angular.z) < min_in_place_vel_th_) res.angular.z = min_in_place_vel_th_ * sign(res.angular.z);
    res.linear.x = 0.0;
    res.linear.y = 0.0;
  }

  ROS_DEBUG("Angular command %f", res.angular.z);
  return res;
}

void PurePursuit::computeVelocities(nav_msgs::Odometry odom)
{
  if(path_.poses.size() == 0)
  {
    ROS_ERROR("path is empty");
  }
  // Get the current robot pose
  tf::Stamped<tf::Pose> robot_pose;
  if(!this->getRobotPose(robot_pose)){
    ROS_ERROR("Can't get robot pose");
    geometry_msgs::Twist empty_twist;
    cmd_vel_ = empty_twist;
  }

  tf::Stamped<tf::Pose> target_pose;
  tf::poseStampedMsgToTF(path_.poses[current_waypoint_],target_pose);

  ROS_DEBUG("current robot pose %f %f ==> %f", robot_pose.getOrigin().x(), robot_pose.getOrigin().y(), tf::getYaw(robot_pose.getRotation()));
  ROS_DEBUG("target robot pose %f %f ==> %f", target_pose.getOrigin().x(), target_pose.getOrigin().y(), tf::getYaw(target_pose.getRotation()));

  geometry_msgs::Twist diff = diff2D(target_pose, robot_pose);
  ROS_DEBUG(" diff %f %f ==> %f", diff.linear.x, diff.linear.y, diff.angular.z);

  geometry_msgs::Twist limit_vel = limitTwist(diff);
  geometry_msgs::Twist test_vel = limit_vel;

  bool legal_traj = true; 
  double scaling_factor = 1.0;
  double ds = scaling_factor / samples_;

  /*if(!legal_traj){
    for(int i = 0; i < samples_; ++i){
      test_vel.linear.x = limit_vel.linear.x * scaling_factor;
      test_vel.linear.y = limit_vel.linear.y * scaling_factor;
      test_vel.angular.z = limit_vel.angular.z * scaling_factor;
      test_vel = limitTwist(test_vel);
      //if(collision_planner_.checkTrajectory(test_vel.linear.x, test_vel.linear.y, test_vel.angular.z, false)){
        legal_traj = true;
        break;
      //}
      scaling_factor -= ds;
    }
  }

  if(!legal_traj){
    ROS_ERROR("Not legal (%.2f, %.2f, %.2f)", limit_vel.linear.x, limit_vel.linear.y, limit_vel.angular.z);
    geometry_msgs::Twist empty_twist;
    cmd_vel_ = empty_twist;
  }*/

  cmd_vel_ = test_vel;
  bool in_goal_position = false;
  while(fabs(diff.linear.x) <= tolerance_trans_ &&
        fabs(diff.linear.y) <= tolerance_trans_ &&
        fabs(diff.angular.z) <= tolerance_rot_)
  {
    if(current_waypoint_ < global_plan_.size() - 1)
    {
      current_waypoint_++;
      tf::poseStampedMsgToTF(global_plan_[current_waypoint_], target_pose);
      diff = diff2D(target_pose, robot_pose);
    }
    else
    {
      ROS_INFO("Reached goal: %d", current_waypoint_);
      in_goal_position = true;
      break;
    }
  }

  //if we're not in the goal position, we need to update time
  if(!in_goal_position)
    goal_reached_time_ = ros::Time::now();

  //check if we've reached our goal for long enough to succeed
  if(goal_reached_time_ + ros::Duration(tolerance_timeout_) < ros::Time::now()){
    geometry_msgs::Twist empty_twist;
    cmd_vel_ = empty_twist;
  }

  pub_vel_.publish(cmd_vel_);
}

void PurePursuit::receivePath(nav_msgs::Path new_path)
{
  if (new_path.header.frame_id == p_global_frame_)
  {
    path_ = new_path;
    current_waypoint_ = 0;
    if (new_path.poses.size() > 0)
    {
      ROS_WARN_STREAM("Received  path succed!");
    }
    else
    {
      ROS_WARN_STREAM("Received empty path!");
    }
  }
  else
  {
    ROS_WARN_STREAM("The path must be published in the " << p_global_frame_
                    << " frame! Ignoring path in " << new_path.header.frame_id
                    << " frame!");
  }
  
}
double PurePursuit::headingDiff(double x, double y, double pt_x, double pt_y, double heading)
  {
    double v1_x = x - pt_x;
    double v1_y = y - pt_y;
    double v2_x = cos(heading);
    double v2_y = sin(heading);

    double perp_dot = v1_x * v2_y - v1_y * v2_x;
    double dot = v1_x * v2_x + v1_y * v2_y;

    //get the signed angle
    double vector_angle = atan2(perp_dot, dot);

    return -1.0 * vector_angle;
  }

geometry_msgs::Twist PurePursuit::diff2D(const tf::Pose& pose1, const tf::Pose& pose2)
{
  geometry_msgs::Twist res;
  tf::Pose diff = pose2.inverse() * pose1;
  res.linear.x = diff.getOrigin().x();
  res.linear.y = diff.getOrigin().y();
  res.angular.z = tf::getYaw(diff.getRotation());

  if(holonomic_ || (fabs(res.linear.x) <= tolerance_trans_ && fabs(res.linear.y) <= tolerance_trans_))
    return res;

  //in the case that we're not rotating to our goal position and we have a non-holonomic robot
  //we'll need to command a rotational velocity that will help us reach our desired heading
  
  //we want to compute a goal based on the heading difference between our pose and the target
  double yaw_diff = headingDiff(pose1.getOrigin().x(), pose1.getOrigin().y(), 
      pose2.getOrigin().x(), pose2.getOrigin().y(), tf::getYaw(pose2.getRotation()));

  //we'll also check if we can move more effectively backwards
  double neg_yaw_diff = headingDiff(pose1.getOrigin().x(), pose1.getOrigin().y(), 
      pose2.getOrigin().x(), pose2.getOrigin().y(), M_PI + tf::getYaw(pose2.getRotation()));

  //check if its faster to just back up
  if(fabs(neg_yaw_diff) < fabs(yaw_diff)){
    ROS_DEBUG("Negative is better: %.2f", neg_yaw_diff);
    yaw_diff = neg_yaw_diff;
  }

  //compute the desired quaterion
  tf::Quaternion rot_diff = tf::createQuaternionFromYaw(yaw_diff);
  tf::Quaternion rot = pose2.getRotation() * rot_diff;
  tf::Pose new_pose = pose1;
  new_pose.setRotation(rot);

  diff = pose2.inverse() * new_pose;
  res.linear.x = diff.getOrigin().x();
  res.linear.y = diff.getOrigin().y();
  res.angular.z = tf::getYaw(diff.getRotation());
  return res;
}

  bool PurePursuit::getRobotPose(tf::Stamped<tf::Pose>& global_pose)
  {

    global_pose.setIdentity();
    tf::Stamped<tf::Pose> robot_pose;
    robot_pose.setIdentity();
    robot_pose.frame_id_ = p_robot_base_frame_;
    robot_pose.stamp_ = ros::Time(0);
    ros::Time current_time = ros::Time::now();

    try{
      tf_->transformPose(p_global_frame_, robot_pose, global_pose);
    }
    catch(tf::LookupException& ex) {
      ROS_ERROR_THROTTLE(1.0, "No Transform available Error looking up robot pose: %s\n", ex.what());
      return false;
    }
    catch(tf::ConnectivityException& ex) {
      ROS_ERROR_THROTTLE(1.0, "Connectivity Error looking up robot pose: %s\n", ex.what());
      return false;
    }
    catch(tf::ExtrapolationException& ex) {
      ROS_ERROR_THROTTLE(1.0, "Extrapolation Error looking up robot pose: %s\n", ex.what());
      return false;
    }

    return true;
  }

/*void PurePursuit::reconfigure(pure_pursuit::PurePursuitConfig &config, uint32_t level)
{
  v_max_ = config.max_linear_velocity;
}*/


int main(int argc, char**argv)
{
  ros::init(argc, argv, "pure_pursuit");

  tf::TransformListener tfl;

  PurePursuit pp;
  pp.initialize(&tfl);

  ros::spin();

  return 0;
}

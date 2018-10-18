/**
 * @file high_level_control.cpp
 * @brief This file provides the implementation of the HighLevelControl class
 *
 * @author Atabak Hafeez [atabakhafeez]
 * @author Maria Ficiu [MariaFiciu]
 * @author Rubin Deliallisi [rdeliallisi]
 * @author Siddharth Shukla [thunderboltsid]
 * @bug No known bugs.
 */

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <cmath>
#include "robot/circle_detect_msg.h"
#include "high_level_control.h"
#include "util_functions.h"
#include "logger.h"
#include <deque>
#define NUM_STORE 5
#define ERROR_THRESHOLD 0.1
#define NUM_CAL 3

std::deque<float> range_right;
std::deque<float> range_left;
std::deque<float> range_center;

typedef struct range_delta{
    int index;
    float delta;

    range_delta(int index_,float delta_){
        index = index_;
        delta = delta_;
    }
}RangeDelta;

bool range_cmp(const RangeDelta& a,const RangeDelta& b){
    return a.delta < b.delta;
}

HighLevelControl::HighLevelControl() : node_() {
    InitialiseMoveSpecs();
    InitialiseMoveStatus();
    InitialiseTopicConnections();
}

void HighLevelControl::InitialiseTopicConnections() {
    bool loaded = true;
    std::string publish_topic, laser_topic, circle_topic;

    if (!node_.getParam("publish_topic",
                        publish_topic)) {
        loaded = false;
    }

    if (!node_.getParam("laser_topic",
                        laser_topic)) {
        loaded = false;
    }

    if (!node_.getParam("circle_topic",
                        circle_topic)) {
        loaded = false;
    }

    if (loaded == false) {
        ROS_INFO("Topics failed to load!");
        ros::shutdown();
    }

    cmd_vel_pub_ = node_.advertise<geometry_msgs::Twist>(publish_topic, 100);
    laser_sub_ = node_.subscribe(laser_topic, 100, &HighLevelControl::LaserCallback, this);
    circle_sub_ = node_.subscribe(circle_topic, 100, &HighLevelControl::CircleCallback, this);
}

void HighLevelControl::InitialiseMoveSpecs() {
    bool loaded = true;

    if (!node_.getParam("high_security_distance",
                        move_specs_.high_security_distance_)) {
        loaded = false;
    }

    if (!node_.getParam("/low_security_distance",
                        move_specs_.low_security_distance_)) {
        loaded = false;
    }

    if (!node_.getParam("/wall_follow_distance",
                        move_specs_.wall_follow_distance_)) {
        loaded = false;
    }

    if (!node_.getParam("/linear_velocity",
                        move_specs_.linear_velocity_)) {
        loaded = false;
    }

    if (!node_.getParam("/angular_velocity",
                        move_specs_.angular_velocity_)) {
        loaded = false;
    }

    if (!node_.getParam("/right_limit",
                        move_specs_.right_limit_)) {
        loaded = false;
    }

    if (!node_.getParam("/left_limit",
                        move_specs_.left_limit_)) {
        loaded = false;
    }

    if (!node_.getParam("/cumulative_angle",
                        move_specs_.cumulative_angle_)) {
        loaded = false;
    }

    move_specs_.turn_type_ = NONE;

    if (loaded == false) {
        ROS_INFO("Parameters failed to load!");
        ros::shutdown();
    }
}

void HighLevelControl::InitialiseMoveStatus() {
    move_status_.can_continue_ = true;
    move_status_.is_close_to_wall_ = false;
    move_status_.is_following_wall_ = false;
    move_status_.circle_hit_mode_ = false;
    move_status_.hit_goal_ = false;
    move_status_.reached_goal_ = false;
    move_status_.count_turn_ = 0;
    move_status_.last_turn_ = 0;
    move_status_.rotate_wall_side_ = 0;
    move_status_.rotate_opposite_side_ = 0;
    move_status_.angle_count_ = 0;

    if (!node_.getParam("/simulation",
                        move_status_.is_sim_)) {
        ROS_INFO("Parameters failed to load!");
        ros::shutdown();
    }
}

void HighLevelControl::LaserCallback(const sensor_msgs::LaserScan::ConstPtr &msg) {
    std::vector<float> ranges(msg->ranges.begin(), msg->ranges.end());
    ROS_INFO("circle_hit_mode_:%d",move_status_.circle_hit_mode_);
    if (!move_status_.circle_hit_mode_) {
        Update(ranges);
        WallFollowMove();
    } else {
        HitCircle(ranges);
    }

    // Log robot status
    ROS_INFO("can_continue:%d, is_following_wall:%d, is_close_to_wall:%d, turn_type:%d\n",
             move_status_.can_continue_, move_status_.is_following_wall_, move_status_.is_close_to_wall_,
             move_specs_.turn_type_);
}

void HighLevelControl::CircleCallback(const robot::circle_detect_msg::ConstPtr& msg) {
    std::vector<float> ranges(msg->ranges.begin(), msg->ranges.end());
    circle_x_ = msg->circle_x;
    circle_y_ = msg->circle_y;
    // If true stay in the mode else check if we can hit circle
    /*move_status_.circle_hit_mode_ = move_status_.circle_hit_mode_ ? true :
                                    CanHit(msg->circle_x, msg->circle_y,
                                           ranges);*/
    move_status_.circle_hit_mode_ = false;
    // Log circle coordinates
    ROS_INFO("circle_x:%lf, circle_y:%lf", circle_x_, circle_y_);
}

bool HighLevelControl::CanHit(double circle_x, double circle_y, std::vector<float>& ranges) {
    // Cannot hit circle if not in wall following mode
    if (move_specs_.turn_type_ == NONE) {
        return false;
    }

    int size = ranges.size();

    // Distance to the wall 20 deg on the opposite side of the circle
    double wall_20;
    // Planar distance to the center of the circle ignoring obstacles
    double center_distance = sqrt(circle_x * circle_x + circle_y * circle_y);
    // Angle to the center of the circle relative to normal Cartesian system
    double center_angle = acos(circle_x / center_distance) / M_PI * 180;
    // Index of the angle in the ranges vector
    int index = static_cast<int>((center_angle + 30) / 240.0 * size);
    // Check if we might get an out of bound index after shifting by 20 deg
    int deg20 = static_cast<int>(20.0 / 240.0 * size);
    if (index < deg20 || index >= size - deg20)
        return false;
    // Distance from LRF in the direction of the circle center
    double center_lrf = ranges[index];
    if (move_specs_.turn_type_ == RIGHT) {
        // Distance 20 deg left from the center
        wall_20 = ranges[index + deg20];
    } else if (move_specs_.turn_type_ == LEFT) {
        // Distance 20 deg right from the center
        wall_20 = ranges[index - deg20];
    } else {
        return false;
    }

    // Threshold to detect if we have fake circle at a corner. If this is the
    // case wall_20 will be smaller than the threshold
    double threshold = center_distance + 1;
    // If the threshold condition is true, if the circle is to the front of the
    // robot and close enough then we go into circle hit mode
    if (wall_20 > threshold && center_lrf < center_distance && circle_x > -0.5
            && circle_x < 0.5 && circle_y < 1 && circle_y > 0) {
        return true;
    }

    return false;
}

void HighLevelControl::Update(std::vector<float>& ranges) {
    double right_min_distance, left_min_distance, center_min_distance;

    int size = ranges.size();
    ROS_INFO("ranges.size:%d",size);
    right_min_distance = GetMin(ranges,0,static_cast<int>(move_specs_.right_limit_ / 90.0 * size));

    center_min_distance = GetMin(ranges, static_cast<int>(move_specs_.right_limit_ / 90.0 * size),
                                 static_cast<int>(move_specs_.left_limit_ / 90.0 * size));

    left_min_distance = GetMin(ranges, static_cast<int>(move_specs_.left_limit_ / 90.0 * size), size);

    ROS_INFO("right:%lf, left:%lf, center:%lf", right_min_distance, left_min_distance, center_min_distance);

      
    if((range_left.size() == NUM_STORE) && (range_right.size() == NUM_STORE) && (range_center.size() == NUM_STORE)){

        float ave_right = 0;
        float ave_left = 0;
        float ave_center = 0;
        
        std::vector<RangeDelta> delta_right;
        std::vector<RangeDelta> delta_left;
        std::vector<RangeDelta> delta_center;

        for(int i = 0;i < NUM_STORE;i++){
            ave_right += range_right[i];
            ave_left += range_left[i];
            ave_center += range_center[i];
        }

        ave_right /= NUM_STORE;
        ave_left /= NUM_STORE;
        ave_center /= NUM_STORE;
       
        for(int j = 0;j < NUM_STORE;j++){

            delta_right.push_back(RangeDelta(j,range_right[j]-ave_right));
            delta_left.push_back(RangeDelta(j,range_left[j]-ave_left));
            delta_center.push_back(RangeDelta(j,range_center[j]-ave_center));
        }

        sort(delta_right.begin(),delta_right.end(),range_cmp);
        sort(delta_left.begin(),delta_left.end(),range_cmp);
        sort(delta_center.begin(),delta_center.end(),range_cmp);
        
        ave_right = 0;
        ave_left = 0;
        ave_center = 0;

        for(int i = 0;i < NUM_CAL;i++){
            ave_right += range_right[delta_right[i].index];
            ave_left += range_left[delta_left[i].index];
            ave_center += range_center[delta_center[i].index];
        }

        ave_right /= NUM_CAL;
        ave_left /= NUM_CAL;
        ave_center /= NUM_CAL;

        ROS_INFO("ave_right:%lf, ave_left:%lf, ave_center:%lf", ave_right, ave_left, ave_center);
        if(fabs(right_min_distance-ave_right) > ERROR_THRESHOLD){
            range_right.push_back(right_min_distance);
            right_min_distance = ave_right;
            range_right.pop_front();
        }
        else{
            range_right.pop_front();
            range_right.push_back(right_min_distance);
        }
        
        if(fabs(left_min_distance-ave_left) > ERROR_THRESHOLD){
            range_left.push_back(left_min_distance);
            left_min_distance = ave_left;
            range_left.pop_front();
        }
        else{
            range_left.pop_front();
            range_left.push_back(left_min_distance);
        }

        if(fabs(center_min_distance-ave_center) > ERROR_THRESHOLD){
            range_center.push_back(center_min_distance);
            center_min_distance = ave_center;
            range_center.pop_front();
        }
        else{
            range_center.pop_front();
            range_center.push_back(center_min_distance);
        }
        

        ROS_INFO("re_right:%lf, re_left:%lf, re_center:%lf", right_min_distance, left_min_distance, center_min_distance);
        CanContinue(right_min_distance, left_min_distance, center_min_distance);

        IsCloseToWall(right_min_distance, left_min_distance, center_min_distance);

    }
    else{
        if(range_left.size() < NUM_STORE)
            range_left.push_back(left_min_distance);

        if(range_right.size() < NUM_STORE)
            range_right.push_back(right_min_distance);

        if(range_center.size() < NUM_STORE)
            range_center.push_back(center_min_distance);
    }
}

void HighLevelControl::CanContinue(double right_min_distance, double left_min_distance,
                                   double center_min_distance) {
    double priority_min, secondary_min;
    if (move_specs_.turn_type_ == RIGHT) {
        priority_min = center_min_distance < right_min_distance ? center_min_distance :
                       right_min_distance;

        secondary_min = left_min_distance;
    } else if (move_specs_.turn_type_ == LEFT) {
        priority_min = center_min_distance < left_min_distance ? center_min_distance :
                       left_min_distance;

        secondary_min = right_min_distance;
    } else {
        priority_min = secondary_min = Min(right_min_distance, left_min_distance,
                                           center_min_distance);
    }

    if (priority_min > move_specs_.high_security_distance_
            && secondary_min > move_specs_.low_security_distance_) {
        move_status_.can_continue_ = true;
    } else {
        move_status_.can_continue_ = false;
    }

    ROS_INFO("priority_min:%lf, secondary_min:%lf", priority_min, secondary_min);
}

void HighLevelControl::IsCloseToWall(double right_min_distance, double left_min_distance,
                                     double center_min_distance) {
    if (move_status_.is_following_wall_) {
        double min;
        if (move_specs_.turn_type_ == RIGHT) {
            min = right_min_distance;
        } else if (move_specs_.turn_type_ == LEFT) {
            min = left_min_distance;
        } else {
            // This case should never happen
            ROS_INFO("Robot has no turn type after being attached to wall!");
            ros::shutdown();
        }

        if (min < move_specs_.wall_follow_distance_) {
            move_status_.is_close_to_wall_ = true;
        } else {
            move_status_.is_close_to_wall_ = false;
        }
    }
}

void HighLevelControl::HitCircle(std::vector<float>& ranges) {

    if (move_status_.hit_goal_) {
        GoToCircle(ranges);
        return;
    }

    AlignRobot(ranges);
}

void HighLevelControl::GoToCircle(std::vector<float>& ranges) {

    // 30 degree in front of the robot to detect if the circle has been hit
    float right_10 = (110.0 / 240.0) * ranges.size();
    float left_10 = (130.0 / 240.0) * ranges.size();
    float center_min = GetMin(ranges, right_10, left_10);

    ROS_INFO("center_min:%f", center_min);

    if (center_min < 0.15) {
        ROS_INFO("Goal Reached!");
        // This is the only instance when the robot does not move after
        // all nodes have been initialized
        move_status_.reached_goal_ = true;
        Move(0, 0);
        return;
    }

    if (move_status_.is_sim_) {
        // Move fast towards goal
        float angular_velocity;
        if (move_specs_.turn_type_ == LEFT) {
            angular_velocity = 0.25;
        } else {
            angular_velocity = -0.25;
        }
        ROS_INFO("Moving towards the goal!");
        Move(move_specs_.linear_velocity_ * 10, angular_velocity);
        return;
    }

    float low_lim = -0.05, high_lim = 0.05;

    if (circle_x_ < -9 || (circle_x_ <= high_lim && circle_x_ >= low_lim)) {
        ROS_INFO("Circle Case: 0!");
        Move(move_specs_.linear_velocity_ , 0);
    } else if (circle_x_ > high_lim && circle_y_ < 1 && circle_y_ > 0) {
        ROS_INFO("Circle Case: 1!");
        Move(0, -move_specs_.angular_velocity_);
    } else if (circle_x_ < low_lim && circle_y_ < 1 && circle_y_ > 0) {
        ROS_INFO("Circle Case: 2!");
        Move(0, move_specs_.angular_velocity_);
    } else {
        ROS_INFO("Circle Case: 3!");
        Move(move_specs_.linear_velocity_ , 0);
    }
}

void HighLevelControl::AlignRobot(std::vector<float>& ranges) {
    int size = ranges.size();

    // 0 deg if right wall and 240 if left wall
    double back_value;
    // 60 deg if right wall and 180 if left wall
    double front_value;
    if (move_specs_.turn_type_ == RIGHT) {
        back_value = ranges[0];
        front_value = ranges[static_cast<int>(20.0 / 60.0 * size) - 1];
    } else if (move_specs_.turn_type_ == LEFT) {
        back_value = ranges[size - 1];
        front_value = ranges[static_cast<int>(40.0 / 60.0 * size) - 1];
    } else {
        // Cannot hit circle if not in wall following mode
        ROS_INFO("The robot has no turn type while trying to align to the wall!\n");
        ros::shutdown();
    }

    // The difference of the values must be very small such that the robot
    // is aligned to the wall it is following. If not turn appropriately.
    double diff = front_value - back_value;
    if (diff <= 0.025 && diff >= -0.025) {
    	ROS_INFO("Aligned to wall!");
        //move_status_.hit_goal_ = true;
    } else if (diff > 0.025) {
    	ROS_INFO("The robot should turn because the difference is not small enough!\n");
        Move(0, -1 * (move_specs_.turn_type_ - 1) * move_specs_.angular_velocity_ / 4);
    } else {
    	ROS_INFO("The robot should turn because the difference is not small enough!\n");
        Move(0, (move_specs_.turn_type_ - 1) * move_specs_.angular_velocity_ / 4);
    }
}

void HighLevelControl::WallFollowMove() {
    if (!move_status_.can_continue_ && !move_status_.is_following_wall_) {
        srand(time(NULL));

        // 50% left mode, 50% right mode
        move_specs_.turn_type_ = rand() % 10000 > 5000 ? RIGHT : LEFT;
        move_status_.is_following_wall_ = true;
    } else if (move_status_.can_continue_ && !move_status_.is_following_wall_) {
    	ROS_INFO("The robot can continue walking!\n");
        Move(move_specs_.linear_velocity_, 0);
    } else {
        if (move_status_.can_continue_ && move_status_.is_close_to_wall_) {
        	ROS_INFO("The robot can continue walking!\n");
            Move(move_specs_.linear_velocity_, 0);

            // Not rotating in place since we are moving forward
            move_status_.rotate_wall_side_ = 0;
            move_status_.rotate_opposite_side_ = 0;

            // Update loop breaking status
            move_status_.last_turn_ = 0;
            move_status_.count_turn_ = 0;
        } else if (!move_status_.can_continue_) {
        	ROS_INFO("The robot can't continue walking!\n");
            Move(0, (move_specs_.turn_type_ - 1) * move_specs_.angular_velocity_);

            // Rotating on the other side relative to the wall we are following
            move_status_.rotate_wall_side_ = 0;
            move_status_.rotate_opposite_side_++;

            // Update loop breaking status
            if (move_status_.last_turn_ == 0 || move_status_.last_turn_ == -1) {
                move_status_.count_turn_++;
            }
            move_status_.last_turn_ = 1;

            move_status_.angle_count_--;
        } else if (!move_status_.is_close_to_wall_) {
        	ROS_INFO("The robot isn't close to wall!\n");
            Move(0, -1 * (move_specs_.turn_type_ - 1) * move_specs_.angular_velocity_);

            // Rotating towards the wall we are following
            move_status_.rotate_opposite_side_ = 0;
            move_status_.rotate_wall_side_++;

            // Update loop breaking status
            if (move_status_.last_turn_ == 0 || move_status_.last_turn_ == 1) {
                move_status_.count_turn_++;
            }
            move_status_.last_turn_ = -1;

            move_status_.angle_count_++;
        }
    }

    BreakLoop();
    BreakRotation();
}

void HighLevelControl::BreakRotation() {
    // In place breaking condition is relative to the angular velocity with
    // which we are turning
    if (move_status_.rotate_wall_side_ > 100 * 1.0 / move_specs_.angular_velocity_ ||
            move_status_.rotate_opposite_side_ > 100 * 1.0 / move_specs_.angular_velocity_) {
        InitialiseMoveStatus();
    }
}

void HighLevelControl::BreakLoop() {
    // In case of a turn loop break out after 5 opposite turns in a row.
    if (move_status_.count_turn_ > 5) {
        ROS_INFO("Stuck in left-right loop!");

        if (move_specs_.turn_type_ == RIGHT) {
            // Short right turn
            ROS_INFO("The robot is turning right!\n");
            Move(move_specs_.linear_velocity_ * 3, -move_specs_.angular_velocity_ * 3);
        } else if (move_specs_.turn_type_ == LEFT) {
            // Short left turn
            ROS_INFO("The robot is turning left!\n");
            Move(move_specs_.linear_velocity_ * 5, move_specs_.angular_velocity_);
        } else {
            // Case should not happen
             ROS_INFO("The turn type is neither right nor left! \n");
             ros::shutdown();
        }
        move_status_.count_turn_ = 0;
    }

    // In case we are in a loop circuit
    if (move_status_.angle_count_ > move_specs_.cumulative_angle_ / move_specs_.angular_velocity_) {
        InitialiseMoveStatus();
        move_status_.angle_count_ = 0;
    }
}

void HighLevelControl::Move(double linear_velocity, double angular_velocity) {
    geometry_msgs::Twist msg;
    ROS_INFO("linear_velocity:%f",linear_velocity);
    ROS_INFO("angular_velocity:%f",angular_velocity);
    msg.linear.x = linear_velocity;
    msg.angular.z = angular_velocity;
    cmd_vel_pub_.publish(msg);
}

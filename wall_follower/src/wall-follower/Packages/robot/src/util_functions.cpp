/**
 * @file util_functions.cpp
 * @brief Defines the utility functions for high_level_control
 *
 * @author Atabak Hafeez [atabakhafeez]
 * @author Maria Ficiu [MariaFiciu]
 * @author Rubin Deliallisi [rdeliallisi]
 * @author Siddharth Shukla [thunderboltsid]
 * @bug No known bugs.
 */

#include "util_functions.h"
#include <vector>
#include <algorithm>
#include <limits>
#include <iostream>
#include <ros/ros.h>

#define LASER_LIMIT 0.6

double GetMin(std::vector<float>& ranges, int start, int finish) {
    if (ranges.size() <= 0 || start < 0 || finish > ranges.size() ||
            start > finish) {
        return 0;
    }

    //std::vector<float>::iterator min = std::min_element(ranges.begin() + start,ranges.begin() + finish);
    //return *min;

    ROS_INFO("range_start:%d",start);
    ROS_INFO("range_finish:%d",finish);
    std::vector<float> range_vec;
    //float min_range = std::numeric_limits<float>::max();
    for(int index = start;index < finish;index++){
        if(!std::isnan(ranges[index])){
            range_vec.push_back(ranges[index]);
        }
    }

    std::sort(range_vec.begin(),range_vec.end());
    ROS_INFO("range_vec_size:%d",(int)range_vec.size());
 
    if(range_vec.size() <= 0)
        return LASER_LIMIT;
    else if(range_vec.size() == 1)
        return range_vec[0];
    else
        return (range_vec[0]+range_vec[1])/2.0;
}


double Min(double right_min_distance, double left_min_distance,
           double center_min_distance) {
    double min;
    if (right_min_distance < center_min_distance) {
        if (right_min_distance < left_min_distance) {
            min = right_min_distance;
        } else {
            min = left_min_distance;
        }
    } else {
        if (center_min_distance < left_min_distance) {
            min = center_min_distance;
        } else {
            min = left_min_distance;
        }
    }
    return min;
}

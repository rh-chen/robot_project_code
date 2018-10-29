#ifndef RAM_PATH_PLANNING_FOLLOW_POSES_HPP
#define RAM_PATH_PLANNING_FOLLOW_POSES_HPP

#include <future>
#include <mutex>

#include <vtkLine.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>

#include <eigen_conversions/eigen_msg.h>
#include <ram_msgs/AdditiveManufacturingTrajectory.h>
#include <ram_path_planning/path_planning_algorithm.hpp>
#include <ram_utils/trajectory_files_manager_1.hpp>
#include <ram_utils/trajectory_files_manager_2.hpp>

namespace ram_path_planning
{
template<class ActionSpec>
  class FollowPoses : public PathPlanningAlgorithm<ActionSpec>
  {
  public:
    // Polygon (possibly multiple contours)
    typedef vtkSmartPointer<vtkPolyData> Polygon;

    FollowPoses();

    std::string generateTrajectory(const Polygon poly_data,
                                   ram_msgs::AdditiveManufacturingTrajectory &msg);

    std::string generateTrajectory(const std::string yaml_file,
                                   ram_msgs::AdditiveManufacturingTrajectory &msg);

    std::string duplicateLayers(ram_msgs::AdditiveManufacturingTrajectory &msg,
                                const unsigned number_of_layers,
                                const double height_between_layers,
                                bool invert_one_of_two_layers);
  };

}

//include the implementation
#include <ram_path_planning/follow_poses_imp.hpp>
#endif

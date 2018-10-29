#ifndef RAM_PATH_PLANNING_FOLLOW_POSES_IMP_HPP
#define RAM_PATH_PLANNING_FOLLOW_POSES_IMP_HPP

namespace ram_path_planning
{
template<class ActionSpec>
  FollowPoses<ActionSpec>::FollowPoses() :
          PathPlanningAlgorithm<ActionSpec>("Follow poses", "Generate a trajectory from a list of poses",
                                            "ram/path_planning/generate_trajectory/follow_poses")
  {
  }

template<class ActionSpec>
  std::string FollowPoses<ActionSpec>::generateTrajectory(const Polygon poly_data,
                                                          ram_msgs::AdditiveManufacturingTrajectory &msg)
  {
    // A contour in the poly_data is a polygon in the ram_msg
    msg.poses.clear();

    vtkIdType n_contours = poly_data->GetNumberOfCells();

    if (n_contours == 0)
      return "YAML is empty";

    for (vtkIdType contour_id(0); contour_id < n_contours; ++contour_id)
    {
      // Contours the poly_data
      vtkIdType n_points = poly_data->GetCell(contour_id)->GetNumberOfPoints();
      if (n_points == 0)
        return "Polygon in YAML file is empty";

      for (vtkIdType point_id(0); point_id < n_points; ++point_id)
      {
        // points in contour
        double p[3]; // Central point
        poly_data->GetCell(contour_id)->GetPoints()->GetPoint(point_id, p);

        ram_msgs::AdditiveManufacturingPose ram_pose;
        ram_pose.unique_id = unique_id::toMsg(unique_id::fromRandom());
        // Geometric pose
        ram_pose.pose.orientation.w = 1;

        ram_pose.pose.position.x = p[0];
        ram_pose.pose.position.y = p[1];
        ram_pose.pose.position.z = p[2];

        // Polygon_start and polygon_end
        if (point_id == 0)
          ram_pose.polygon_start = true;
        if ((point_id + 1) == n_points)
          ram_pose.polygon_end = true;

        msg.poses.push_back(ram_pose);
      }
    }

    return "";
  }

template<class ActionSpec>
  std::string FollowPoses<ActionSpec>::duplicateLayers(ram_msgs::AdditiveManufacturingTrajectory &msg,
                                                       const unsigned number_of_layers,
                                                       const double height_between_layers,
                                                       bool invert_one_of_two_layers)
  {
    if (number_of_layers < 2)
      return "Number of layers < 2";

    if (!msg.poses.empty())
    {
      msg.poses.front().polygon_start = true;
      msg.poses.back().polygon_end = true;
    }

    const std::vector<ram_msgs::AdditiveManufacturingPose> first_layer(msg.poses);

    // msg contains only one layer, copy it
    for (unsigned i(1); i < number_of_layers; ++i)
    {
      std::vector < ram_msgs::AdditiveManufacturingPose > new_layer(first_layer);
      if (invert_one_of_two_layers && i % 2 == 1)
      {
        // Invert layer before pushing it
        std::reverse(new_layer.begin(), new_layer.end());
        new_layer.front().polygon_start = true;
        new_layer.front().polygon_end = false;

        new_layer.back().polygon_end = true;
        new_layer.back().polygon_start = false;
      }

      for (auto &pose : new_layer)
      {
        pose.unique_id = unique_id::toMsg(unique_id::fromRandom());
        pose.layer_level = i;
        pose.layer_index = pose.layer_level;
        pose.pose.position.z += i * height_between_layers;
      }

      // Insert the poses
      msg.poses.insert(msg.poses.end(), new_layer.begin(), new_layer.end());
    }

    if (invert_one_of_two_layers)
      msg.similar_layers = false;
    else
      msg.similar_layers = true;
    return "";
  }

template<class ActionSpec>
  std::string FollowPoses<ActionSpec>::generateTrajectory(const std::string yaml_file,
                                                          ram_msgs::AdditiveManufacturingTrajectory &msg)
  {
    if (ram_utils::yamlFileToTrajectory(yaml_file, msg))
      return "";

    // Prepare contours
    const Polygon poly_data = Polygon::New();
    std::vector<unsigned> layer_count;
    if (ram_utils::yamlFileToPolydata2(yaml_file, poly_data, layer_count))
    {
      std::string error = generateTrajectory(poly_data, msg);
      if (!error.empty())
        return error;

      msg.file = yaml_file;
      msg.generated = ros::Time::now();
      msg.modified = msg.generated;
      msg.similar_layers = false;
      msg.generation_info = "From YAML (points) file";
      return "";
    }

    return "Failed to read YAML file";
  }
}

#endif

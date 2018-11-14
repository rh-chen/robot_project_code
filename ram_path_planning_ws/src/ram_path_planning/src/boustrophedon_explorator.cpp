#include <ipa_room_exploration/boustrophedon_explorator.h>

BoustrophedonExplorer::BoustrophedonExplorer()
{

}

void BoustrophedonExplorer::getExplorationPath(const cv::Mat& room_map, std::vector<geometry_msgs::Pose2D>& path,
		const float map_resolution, const cv::Point starting_position, const cv::Point2d map_origin,
		const double grid_spacing_in_pixel, const double path_eps, const bool plan_for_footprint,
		const Eigen::Matrix<float, 2, 1> robot_to_fov_vector, const double min_cell_area)
{
	ROS_INFO("Planning the boustrophedon path trough the room.");

	// *********************** I. Find the main directions of the map and rotate it in this manner. ***********************
	// *********************** II. Sweep a slice trough the map and mark the found cell boundaries. ***********************
	// *********************** III. Find the separated cells. ***********************
	cv::Mat R;
	cv::Rect bbox;
	cv::Mat rotated_room_map;
	std::vector<GeneralizedPolygon> cell_polygons;
	std::vector<cv::Point> polygon_centers;

	computeCellDecompositionWithRotation(room_map,
										 map_resolution
										 min_cell_area,
										 0.,
										 R,
										 bbox,
										 rotated_room_map,
										 cell_polygons,
										 polygon_centers);

	ROS_INFO("Found the cells in the given map.");

	// *********************** IV. Determine the cell paths. ***********************
	// determine the start cell that contains the start position
	std::vector<cv::Point> starting_point_vector(1, starting_position); // opencv syntax
	cv::transform(starting_point_vector, starting_point_vector, R);
	const cv::Point rotated_starting_point = starting_point_vector[0]; // point that keeps track of the last point after the boustrophedon path in each cell

	// go trough the cells [in optimal visiting order] and determine the boustrophedon paths
	ROS_INFO("Starting to get the paths for each cell, number of cells: %d", (int)cell_polygons.size());
	const int grid_spacing_as_int = (int)std::floor(grid_spacing_in_pixel); // convert fov-radius to int
	const int half_grid_spacing_as_int = (int)std::floor(0.5*grid_spacing_in_pixel); // convert fov-radius to int
	std::cout << "Boustrophedon grid_spacing_as_int=" << grid_spacing_as_int << std::endl;
	cv::Point robot_pos = rotated_starting_point;	// point that keeps track of the last point after the boustrophedon path in each cell
	std::vector<cv::Point> fov_middlepoint_path;	// this is the trajectory of centers of the robot footprint or the field of view

	for(size_t cell=0; cell<cell_polygons.size(); ++cell)
	{
		computeBoustrophedonPath(rotated_room_map,
								 map_resolution,
								 cell_polygons[optimal_order[cell]],
								 fov_middlepoint_path,
								 robot_pos,
								 grid_spacing_as_int,
								 half_grid_spacing_as_int,
								 path_eps);
	}

	// transform the calculated path back to the originally rotated map and create poses with an angle
	RoomRotator room_rotation;
	std::vector<geometry_msgs::Pose2D> fov_poses;//this is the trajectory of poses of the robot footprint or the field of view,in[pixels]
	room_rotation.transformPathBackToOriginalRotation(fov_middlepoint_path, fov_poses, R);
	ROS_INFO("Found the cell paths.");

	// if the path should be planned for the robot footprint create the path and return here
	if (plan_for_footprint == true)
	{
		for(std::vector<geometry_msgs::Pose2D>::iterator pose=fov_poses.begin(); pose != fov_poses.end(); ++pose)
		{
			geometry_msgs::Pose2D current_pose;
			current_pose.x = (pose->x * map_resolution) + map_origin.x;
			current_pose.y = (pose->y * map_resolution) + map_origin.y;
			current_pose.theta = pose->theta;
			path.push_back(current_pose);
		}
		return;
	}
}


void BoustrophedonExplorer::findBestCellDecomposition(const cv::Mat& room_map, const float map_resolution, const double min_cell_area,
		cv::Mat& R, cv::Rect& bbox, cv::Mat& rotated_room_map,
		std::vector<GeneralizedPolygon>& cell_polygons, std::vector<cv::Point>& polygon_centers)
{
	// *********************** I. Find the main directions of the map and rotate it in this manner. ***********************
	// *********************** II. Sweep a slice trough the map and mark the found cell boundaries. ***********************
	// *********************** III. Find the separated cells. ***********************
	cv::Mat R_1, R_2;
	cv::Rect bbox_1, bbox_2;
	cv::Mat rotated_room_map_1, rotated_room_map_2;
	std::vector<GeneralizedPolygon> cell_polygons_1, cell_polygons_2;
	std::vector<cv::Point> polygon_centers_1, polygon_centers_2;
	computeCellDecompositionWithRotation(room_map, map_resolution, min_cell_area, 0., R_1, bbox_1, rotated_room_map_1, cell_polygons_1, polygon_centers_1);
	computeCellDecompositionWithRotation(room_map, map_resolution, min_cell_area, 90./180.*CV_PI, R_2, bbox_2, rotated_room_map_2, cell_polygons_2, polygon_centers_2);

	// select the cell decomposition with good axis alignment which produces less cells
	if (cell_polygons_1.size() <= cell_polygons_2.size())
	{
		R = R_1;
		bbox = bbox_1;
		rotated_room_map = rotated_room_map_1;
		cell_polygons = cell_polygons_1;
		polygon_centers = polygon_centers_1;
	}
	else
	{
		R = R_2;
		bbox = bbox_2;
		rotated_room_map = rotated_room_map_2;
		cell_polygons = cell_polygons_2;
		polygon_centers = polygon_centers_2;
	}
}

void BoustrophedonExplorer::computeCellDecompositionWithRotation(const cv::Mat& room_map, const float map_resolution, const double min_cell_area,
		const double rotation_offset, cv::Mat& R, cv::Rect& bbox, cv::Mat& rotated_room_map,
		std::vector<GeneralizedPolygon>& cell_polygons, std::vector<cv::Point>& polygon_centers)
{
	// *********************** I. Find the main directions of the map and rotate it in this manner. ***********************
	RoomRotator room_rotation;
	room_rotation.computeRoomRotationMatrix(room_map, R, bbox, map_resolution, 0, rotation_offset);
	room_rotation.rotateRoom(room_map, rotated_room_map, R, bbox);

	// *********************** II. Sweep a slice trough the map and mark the found cell boundaries. ***********************
	// *********************** III. Find the separated cells. ***********************
	computeCellDecomposition(rotated_room_map, map_resolution, min_cell_area, cell_polygons, polygon_centers);
}

void BoustrophedonExplorer::computeCellDecomposition(const cv::Mat& room_map, const float map_resolution, const double min_cell_area,
		std::vector<GeneralizedPolygon>& cell_polygons, std::vector<cv::Point>& polygon_centers)
{
	// *********************** II. Sweep a slice trough the map and mark the found cell boundaries. ***********************
	// create a map copy to mark the cell boundaries
	cv::Mat cell_map = room_map.clone();

	// find smallest y-value for that a white pixel occurs, to set initial y value and find initial number of segments
	size_t y_start = 0;
	int n_start = 0;
	bool found = false, obstacle = false;
	for(size_t y=0; y<room_map.rows; ++y)
	{
		for(size_t x=0; x<room_map.cols; ++x)
		{
			if(room_map.at<uchar>(y,x) == 255 && found == false)
			{
				y_start = y;
				found = true;
			}
			else if(found == true && obstacle == false && room_map.at<uchar>(y,x) == 0)
			{
				++n_start;
				obstacle = true;
			}
			else if(found == true && obstacle == true && room_map.at<uchar>(y,x) == 255)
			{
				obstacle = false;
			}
		}

		if(found == true)
			break;
	}

	// swipe trough the map and detect critical points
	int previous_number_of_segments = n_start;
	for(size_t y=y_start+1; y<room_map.rows; ++y) // start at y_start+1 because we know number of segments at y_start
	{
		int number_of_segments = 0; // int to count how many segments at the current slice are
		bool obstacle_hit = false; // bool to check if the line currently hit an obstacle, s.t. not all black pixels trigger an event
		bool hit_white_pixel = false; // bool to check if a white pixel has been hit at the current slice, to start the slice at the first white pixel

		// count number of segments within this row
		for(size_t x=0; x<room_map.cols; ++x)
		{
			if(room_map.at<uchar>(y,x) == 255 && hit_white_pixel == false)
				hit_white_pixel = true;
			else if(hit_white_pixel == true)
			{
				if(obstacle_hit == false && room_map.at<uchar>(y,x) == 0) // check for obstacle
				{
					++number_of_segments;
					obstacle_hit = true;
				}
				else if(obstacle_hit == true && room_map.at<uchar>(y,x) == 255) // check for leaving obstacle
				{
					obstacle_hit = false;
				}
			}
		}

		// reset hit_white_pixel to use this Boolean later
		hit_white_pixel = false;

		// check if number of segments has changed --> event occurred
		if(previous_number_of_segments < number_of_segments) // IN event
		{
			// check the current slice again for critical points
			for(int x=0; x<room_map.cols; ++x)
			{
				if(room_map.at<uchar>(y,x) == 255 && hit_white_pixel == false)
					hit_white_pixel = true;
				else if(hit_white_pixel == true && room_map.at<uchar>(y,x) == 0)
				{
					// check over black pixel for other black pixels, if none occur a critical point is found
					bool critical_point = true;
					for(int dx=-1; dx<=1; ++dx)
						if(room_map.at<uchar>(y-1,x+dx) == 0)
							critical_point = false;

					// if a critical point is found mark the separation, note that this algorithm goes left and right
					// starting at the critical point until an obstacle is hit, because this prevents unnecessary cells
					// behind other obstacles on the same y-value as the critical point
					if(critical_point == true)
					{
						// to the left until a black pixel is hit
						for(int dx=-1; x+dx>0; --dx)
						{
							if(cell_map.at<uchar>(y,x+dx) == 255)
								cell_map.at<uchar>(y,x+dx) = 0;
							else if(cell_map.at<uchar>(y,x+dx) == 0)
								break;
						}

						// to the right until a black pixel is hit
						for(int dx=1; x+dx<room_map.cols; ++dx)
						{
							if(cell_map.at<uchar>(y,x+dx) == 255)
								cell_map.at<uchar>(y,x+dx) = 0;
							else if(cell_map.at<uchar>(y,x+dx) == 0)
								break;
						}
					}
				}
			}
		}
		else if(previous_number_of_segments > number_of_segments) // OUT event
		{
			// check the previous slice again for critical points --> y-1
			for(int x=0; x<room_map.cols; ++x)
			{
				if(room_map.at<uchar>(y-1,x) == 255 && hit_white_pixel == false)
					hit_white_pixel = true;
				else if(hit_white_pixel == true && room_map.at<uchar>(y-1,x) == 0)
				{
					// check over black pixel for other black pixels, if none occur a critical point is found
					bool critical_point = true;
					for(int dx=-1; dx<=1; ++dx)
						if(room_map.at<uchar>(y,std::min(x+dx, room_map.cols-1)) == 0) // check at side after obstacle
							critical_point = false;

					// if a critical point is found mark the separation, note that this algorithm goes left and right
					// starting at the critical point until an obstacle is hit, because this prevents unnecessary cells
					// behind other obstacles on the same y-value as the critical point
					if(critical_point == true)
					{
						// to the left until a black pixel is hit
						for(int dx=-1; x+dx>0; --dx)
						{
							if(cell_map.at<uchar>(y-1,x+dx) == 255)
								cell_map.at<uchar>(y-1,x+dx) = 0;
							else if(cell_map.at<uchar>(y-1,x+dx) == 0)
								break;
						}

						// to the right until a black pixel is hit
						for(int dx=1; x+dx<room_map.cols; ++dx)
						{
							if(cell_map.at<uchar>(y-1,x+dx) == 255)
								cell_map.at<uchar>(y-1,x+dx) = 0;
							else if(cell_map.at<uchar>(y-1,x+dx) == 0)
								break;
						}
					}
				}
			}
		}

		// save the found number of segments
		previous_number_of_segments = number_of_segments;
	}

#ifdef DEBUG_VISUALIZATION
	cv::imshow("cell_map", cell_map);
#endif


	// *********************** III. Find the separated cells. ***********************
	std::vector<std::vector<cv::Point> > cells;
	cv::Mat cell_copy = cell_map.clone();
	correctThinWalls(cell_copy);	// just adds a few obstacle pixels to avoid merging independent segments
	//cv::findContours(cell_copy, cells, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
	cv::findContours(cell_copy, cells, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

	// create generalized Polygons out of the contours to handle the cells
	for(size_t cell=0; cell<cells.size(); ++cell)
	{
		if(cv::contourArea(cells[cell])>=min_cell_area)
		{
			GeneralizedPolygon current_cell(cells[cell], map_resolution);
			cell_polygons.push_back(current_cell);
			polygon_centers.push_back(current_cell.getCenter());
		}
	}
}

void BoustrophedonExplorer::correctThinWalls(cv::Mat& room_map)
{
	for (int v=1; v<room_map.rows; ++v)
	{
		for (int u=1; u<room_map.cols; ++u)
		{
			if (room_map.at<uchar>(v-1,u-1)==255 \
				&& room_map.at<uchar>(v-1,u)==0 \
				&& room_map.at<uchar>(v,u-1)==0 \
				&& room_map.at<uchar>(v,u)==255)
				room_map.at<uchar>(v,u)=0;
			else if (room_map.at<uchar>(v-1,u-1)==0 \
					 && room_map.at<uchar>(v-1,u)==255 \
					 && room_map.at<uchar>(v,u-1)==255 \
					 && room_map.at<uchar>(v,u)==0)
				room_map.at<uchar>(v,u-1)=0;
		}
	}
}

void BoustrophedonExplorer::computeBoustrophedonPath(const cv::Mat& room_map, const float map_resolution, const GeneralizedPolygon& cell,
		std::vector<cv::Point>& fov_middlepoint_path, cv::Point& robot_pos,
		const int grid_spacing_as_int, const int half_grid_spacing_as_int, const double path_eps)
{
	//  get a map that has only the current cell drawn in
	//	Remark:	single cells are obstacle free so it is sufficient to use the cell to check if a position can be reached during the
	//  execution of the coverage path
	cv::Mat cell_map;
	cell.drawPolygon(cell_map, cv::Scalar(255));

	// align the longer dimension of the cell horizontally with the x-axis
	cv::Point cell_center = cell.getCenter();
	cv::Mat R_cell;
	cv::Rect cell_bbox;
	cv::Mat rotated_cell_map;
	RoomRotator cell_rotation;
	cell_rotation.computeRoomRotationMatrix(cell_map, R_cell, cell_bbox, map_resolution, &cell_center);
	cell_rotation.rotateRoom(cell_map, rotated_cell_map, R_cell, cell_bbox);

	// create inflated obstacles room map and rotate according to cell
	//  --> used later for checking accessibility of Boustrophedon path inside the cell
	cv::Mat inflated_room_map, rotated_inflated_room_map;
	cv::erode(room_map, inflated_room_map, cv::Mat(), cv::Point(-1, -1), half_grid_spacing_as_int);
	cell_rotation.rotateRoom(inflated_room_map, rotated_inflated_room_map, R_cell, cell_bbox);
	cv::Mat rotated_inflated_cell_map = rotated_cell_map.clone();
	for (int v=0; v<rotated_inflated_cell_map.rows; ++v)
		for (int u=0; u<rotated_inflated_cell_map.cols; ++u)
			if (rotated_inflated_cell_map.at<uchar>(v,u)!=0 && rotated_inflated_room_map.at<uchar>(v,u)==0)
				rotated_inflated_cell_map.at<uchar>(v,u) = 128;

	// compute the basic Boustrophedon grid lines
	BoustrophedonGrid grid_lines;
	GridGenerator::generateBoustrophedonGrid(rotated_cell_map,
											 rotated_inflated_cell_map,
											 -1,
											 grid_lines,
											 cv::Vec4i(0, 0, 0, 0), 
											 //cv::Vec4i(min_x, max_x, min_y, max_y),
											 grid_spacing_as_int,
											 half_grid_spacing_as_int, 1);
	
	// if no edge could be found in the cell (e.g. if it is too small), ignore it
	if(grid_lines.size()==0)
		return;

	// get the edge nearest to the current robot position to start the boustrophedon path at by looking at the
	// upper and lower horizontal path (possible nearest locations) for the edges transformed to the original coordinates (easier)
	std::vector<cv::Point> outer_corners(4);
	outer_corners[0] = grid_lines[0].upper_line[0];		// upper left corner
	outer_corners[1] = grid_lines[0].upper_line.back();	// upper right corner
	outer_corners[2] = grid_lines.back().upper_line[0];	// lower left corner
	outer_corners[3] = grid_lines.back().upper_line.back();	// lower right corner
	cv::Mat R_cell_inv;
	cv::invertAffineTransform(R_cell, R_cell_inv);	// invert the rotation matrix to remap the determined points to the original cell
	cv::transform(outer_corners, outer_corners, R_cell_inv);
	double min_corner_dist = path_planner_.planPath(room_map, robot_pos, outer_corners[0], 1.0, 0.0, map_resolution);
	int min_corner_index = 0;
	for (int i=1; i<4; ++i)
	{
		double dist = path_planner_.planPath(room_map, robot_pos, outer_corners[i], 1.0, 0.0, map_resolution);
		if (dist < min_corner_dist)
		{
			min_corner_dist = dist;
			min_corner_index = i;
		}
	}
	bool start_from_upper_path = (min_corner_index<2 ? true : false)
	// boolean to determine on which side the path should start and to check where the path ended;
	bool start_from_left = (min_corner_index%2==0 ? true : false); 
	
	cv::Point cell_robot_pos;
	bool start = true;
	std::vector<cv::Point> current_fov_path;
	if(start_from_upper_path == true) // plan the path starting from upper horizontal line
	{
		for(BoustrophedonGrid::iterator line=grid_lines.begin(); line!=grid_lines.end(); ++line)
		{
			if(start == true) // at the beginning of path planning start at first horizontal line --> no vertical points between lines
			{
				if(start_from_left == true)
					cell_robot_pos = line->upper_line[0];
				else
					cell_robot_pos = line->upper_line.back();
				start = false;
			}

			if(start_from_left == true) // plan path from left to right corner
			{
				// get points on transition between horizontal lines by using the Astar-path
				std::vector<cv::Point> astar_path;
				path_planner_.planPath(rotated_inflated_cell_map, cell_robot_pos, line->upper_line[0], 1.0, 0.0, map_resolution, 0, &astar_path);
				downsamplePath(astar_path, current_fov_path, cell_robot_pos, path_eps);

				// get points between left and right corner
				downsamplePath(line->upper_line, current_fov_path, cell_robot_pos, path_eps);

				// add the lower path of the current line if available (and then start from the left side again next time)
				if (line->has_two_valid_lines == true)
					downsamplePathReverse(line->lower_line, current_fov_path, cell_robot_pos, path_eps);
				else
					start_from_left = false;	// start from the right side next time
			}
			else // plan path from right to left corner
			{
				// get points on transition between horizontal lines by using the Astar-path
				std::vector<cv::Point> astar_path;
				path_planner_.planPath(rotated_inflated_cell_map, cell_robot_pos, line->upper_line.back(), 1.0, 0.0, map_resolution, 0, &astar_path);
				downsamplePath(astar_path, current_fov_path, cell_robot_pos, path_eps);

				// get points between right and left corner
				downsamplePathReverse(line->upper_line, current_fov_path, cell_robot_pos, path_eps);

				// add the lower path of the current line if available (and then start from the right side again next time)
				if (line->has_two_valid_lines == true)
					downsamplePath(line->lower_line, current_fov_path, cell_robot_pos, path_eps);
				else
					start_from_left = true;	// start from the left side next time
			}
		}
	}
	else // plan the path from the lower horizontal line
	{
		for(BoustrophedonGrid::reverse_iterator line=grid_lines.rbegin(); line!=grid_lines.rend(); ++line)
		{
			if(start == true) // at the beginning of path planning start at first horizontal line --> no vertical points between lines
			{
				if(start_from_left == true)
					cell_robot_pos = line->upper_line[0];
				else
					cell_robot_pos = line->upper_line.back();
				start = false;
			}

			if(start_from_left == true) // plan path from left to right corner
			{
				// get points on transition between horizontal lines by using the Astar-path
				std::vector<cv::Point> astar_path;
				path_planner_.planPath(rotated_inflated_cell_map, cell_robot_pos, line->upper_line[0], 1.0, 0.0, map_resolution, 0, &astar_path);
				downsamplePath(astar_path, current_fov_path, cell_robot_pos, path_eps);

				// get points between left and right corner
				downsamplePath(line->upper_line, current_fov_path, cell_robot_pos, path_eps);

				// add the lower path of the current line if available (and then start from the left side again next time)
				if (line->has_two_valid_lines == true)
					downsamplePathReverse(line->lower_line, current_fov_path, cell_robot_pos, path_eps);
				else
					start_from_left = false;	// start from the right side next time
			}
			else // plan path from right to left corner
			{
				// get points on transition between horizontal lines by using the Astar-path
				std::vector<cv::Point> astar_path;
				path_planner_.planPath(rotated_inflated_cell_map, cell_robot_pos, line->upper_line.back(), 1.0, 0.0, map_resolution, 0, &astar_path);
				downsamplePath(astar_path, current_fov_path, cell_robot_pos, path_eps);

				// get points between right and left corner
				downsamplePathReverse(line->upper_line, current_fov_path, cell_robot_pos, path_eps);

				// add the lower path of the current line if available (and then start from the right side again next time)
				if (line->has_two_valid_lines == true)
					downsamplePath(line->lower_line, current_fov_path, cell_robot_pos, path_eps);
				else
					start_from_left = true;	// start from the left side next time
			}
		}
	}

	// remap the fov path to the originally rotated cell and add the found points to the global path
	cv::transform(current_fov_path, current_fov_path, R_cell_inv);
	for(std::vector<cv::Point>::iterator point=current_fov_path.begin(); point!=current_fov_path.end(); ++point)
		fov_middlepoint_path.push_back(*point);

	// also update the current robot position
	std::vector<cv::Point> current_pos_vector(1, cell_robot_pos);
	cv::transform(current_pos_vector, current_pos_vector, R_cell_inv);
	robot_pos = current_pos_vector[0];
}

void BoustrophedonExplorer::downsamplePath(const std::vector<cv::Point>& original_path, std::vector<cv::Point>& downsampled_path,
		cv::Point& robot_pos, const double path_eps)
{
	// downsample path
	for(size_t path_point=0; path_point<original_path.size(); ++path_point)
	{
		if(cv::norm(robot_pos-original_path[path_point]) >= path_eps)
		{
			downsampled_path.push_back(original_path[path_point]);
			robot_pos = original_path[path_point];
		}
	}
	// add last element
	if (original_path.size() > 0)
	{
		downsampled_path.push_back(original_path.back());
		robot_pos = original_path.back();
	}
}

void BoustrophedonExplorer::downsamplePathReverse(const std::vector<cv::Point>& original_path, std::vector<cv::Point>& downsampled_path,
		cv::Point& robot_pos, const double path_eps)
{
	// downsample path
	for(size_t path_point=original_path.size()-1; ; --path_point)
	{
		if(cv::norm(robot_pos-original_path[path_point]) >= path_eps)
		{
			downsampled_path.push_back(original_path[path_point]);
			robot_pos = original_path[path_point];
		}
		if (path_point == 0)
			break;
	}
	// add last element
	if (original_path.size() > 0)
	{
		downsampled_path.push_back(original_path[0]);
		robot_pos = original_path[0];
	}
}

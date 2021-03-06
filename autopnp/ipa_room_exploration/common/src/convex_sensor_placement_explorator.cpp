#include <ipa_room_exploration/convex_sensor_placement_explorator.h>

// Constructor
convexSPPExplorator::convexSPPExplorator()
{

}

// Function that creates a Qsopt optimization problem and solves it, using the given matrices and vectors.
template<typename T>
void convexSPPExplorator::solveOptimizationProblem(std::vector<T>& C, const cv::Mat& V, const std::vector<double>* W)
{
	// initialize the problem
	CoinModel problem_builder;

	ROS_INFO("Creating and solving linear program.");

	// add the optimization variables to the problem
	int rval;
	for(size_t variable=0; variable<C.size(); ++variable)
	{
		if(W != NULL) // if a weight-vector is provided, use it to set the weights for the var>iables
		{
			problem_builder.setColBounds(variable, 0.0, 1.0);
			problem_builder.setObjective(variable, W->operator[](variable));
		}
		else
		{
			problem_builder.setColBounds(variable, 0.0, 1.0);
			problem_builder.setObjective(variable, 1.0);
			problem_builder.setInteger(variable);
		}

	}

	// inequality constraints to ensure that every position has been seen at least once
	for(size_t row=0; row<V.rows; ++row)
	{
		// gather the indices of the variables that are used in this constraint (row), i.e. where V[row][column] == 1
		std::vector<int> variable_indices;
		for(size_t col=0; col<V.cols; ++col)
			if(V.at<uchar>(row, col) == 1)
				variable_indices.push_back((int) col);

		// all indices are 1 in this constraint
		std::vector<double> variable_coefficients(variable_indices.size(), 1.0);

		// add the constraint
		problem_builder.addRow((int) variable_indices.size(), &variable_indices[0], &variable_coefficients[0], 1.0);
	}

	// load the created LP problem to the solver
	OsiClpSolverInterface LP_solver;
	OsiClpSolverInterface* solver_pointer = &LP_solver;

	solver_pointer->loadFromCoinModel(problem_builder);

	// testing
	solver_pointer->writeLp("lin_cpp_prog", "lp");

	// solve the created optimization problem
	CbcModel model(*solver_pointer);
	model.solver()->setHintParam(OsiDoReducePrint, true, OsiHintTry);

//	CbcHeuristicLocal heuristic2(model);
//	model.addHeuristic(&heuristic2);

	model.initialSolve();
	model.branchAndBound();

	// retrieve solution
	const double * solution = model.solver()->getColSolution();

	for(size_t res=0; res<C.size(); ++res)
	{
//		std::cout << solution[i] << std::endl;
		C[res] = solution[res];
	}
}

// Function that is used to get a coverage path that covers the free space of the given map. It is programmed according to
//
//   Arain, M. A., Cirillo, M., Bennetts, V. H., Schaffernicht, E., Trincavelli, M., & Lilienthal, A. J. (2015, May).
//   Efficient measurement planning for remote gas sensing with mobile robots.
//   In 2015 IEEE International Conference on Robotics and Automation (ICRA) (pp. 3428-3434). IEEE.
//
// In this paper a linear program is used to get the minimal set of sensing poses to check the whole area for gas, but it
// can also be used to to do coverage path planning by defining the visibility function in a certain way. To do so the
// following steps are done
//	I.	Discretize the given map into cells by using the given cell size. The free cells are the ones that have a center
//		that is a white pixel in the map (value=255), i.e. belongs to the free space. After this compute the candidate
//		sensing poses, that determine the possible poses the robot later is.
//	II.	Construct the matrices that are used in the linear program. These are:
//			W: weight matrix for the re-weighted convex relaxation method
//			V: visibility matrix that stores 1 if cell i can be observed from candidate pose j (V[i,j] = 1) or 0 else
//				(when planning for footprint: if cell i can be covered from pose j)
// III.	Solve the optimization problems in the following order
//		1. 	Iteratively solve the weighted optimization problem to approximate the problem by a convex optimization. This
//			speeds up the solution and is done until the sparsity of the optimization variables doesn't change anymore,
//			i.e. converged, or a specific number of iterations is reached. To measure the sparsity a l^0_eps measure is
//			used, that checks |{i: c[i] <= eps}|. In each step the weights are adapted with respect to the previous solution.
//		2.	After the convex relaxation procedure is finished the candidate poses that are not chosen previously, i.e.
//			those that have an corresponding optimization variable equal to 0, are discarded and the matrices of the
//			optimization problem are updated. With the reduced problem the original unweighted problem is solved to get
//			the final solution, that shows which candidate poses should be visited to observe/visit the whole free space.
//	IV.	Read out the chosen sensing poses (those corresponding to a variable of the solution equal to one) and create a
//		path trough all of them. The path is created by applying a repetitive nearest neighbor algorithm. This algorithm
//		solves a TSP for the chosen poses, with each pose being the start node once. Out of the computed paths then the
//		shortest is chosen, which is an Hamiltonian cycle through the graph. After this path has been obtained, determine
//		the pose in the cycle that is closest to the start position, which becomes the start of the fov-path.
void convexSPPExplorator::getExplorationPath(const cv::Mat& room_map, std::vector<geometry_msgs::Pose2D>& path,
		const float map_resolution, const cv::Point starting_position, const cv::Point2d map_origin,
		const int cell_size_pixel, const double delta_theta, const std::vector<Eigen::Matrix<float, 2, 1> >& fov_corners_meter,
		const Eigen::Matrix<float, 2, 1>& robot_to_fov_vector_meter, const double largest_robot_to_footprint_distance_meter,
		const uint sparsity_check_range, const bool plan_for_footprint)
{
	const int half_cell_size = cell_size_pixel/2;

	// *********************** I. Find the main directions of the map and rotate it in this manner. ***********************
	// rotate map
	cv::Mat R;
	cv::Rect bbox;
	cv::Mat rotated_room_map;
	RoomRotator room_rotation;
	const double room_rotation_angle = room_rotation.computeRoomRotationMatrix(room_map, R, bbox, map_resolution);
	room_rotation.rotateRoom(room_map, rotated_room_map, R, bbox);

	// transform starting position
	std::vector<cv::Point> starting_point_vector(1, starting_position); // opencv syntax
	cv::transform(starting_point_vector, starting_point_vector, R);
	cv::Point rotated_starting_position = starting_point_vector[0];

	// ************* II. Go trough the map and discretize it. *************
	// get cells
	// compute the basic Boustrophedon grid lines
	std::vector<cv::Point> cell_centers_rotated, cell_centers;	// in [pixels]
	cv::Mat inflated_rotated_room_map;
	BoustrophedonGrid grid_lines;
	GridGenerator::generateBoustrophedonGrid(rotated_room_map, inflated_rotated_room_map, half_cell_size, grid_lines, cv::Vec4i(0, 0, 0, 0),
			cell_size_pixel, half_cell_size, cell_size_pixel);
	// convert grid points format
	for (BoustrophedonGrid::iterator line=grid_lines.begin(); line!=grid_lines.end(); ++line)
	{
		cell_centers_rotated.insert(cell_centers_rotated.end(), line->upper_line.begin(), line->upper_line.end());
		if (line->has_two_valid_lines == true)
			cell_centers_rotated.insert(cell_centers_rotated.end(), line->lower_line.begin(), line->lower_line.end());
	}
	// rotate back to original image
	cv::Mat R_inv;
	cv::invertAffineTransform(R, R_inv);
	std::vector<cv::Point> fov_middlepoint_path_transformed;
	cv::transform(cell_centers_rotated, cell_centers, R_inv);

//	// print grid
//	cv::Mat point_map = room_map.clone();
//	for(std::vector<cv::Point>::iterator point = cell_centers.begin(); point != cell_centers.end(); ++point)
//	{
//		cv::circle(point_map, *point, 2, cv::Scalar(127), CV_FILLED);
//		std::cout << "  - " << *point << "\n";
//	}
//	cv::imshow("grid", point_map);
//	cv::waitKey();

	// get candidate sensing poses
	std::vector<geometry_msgs::Pose2D> candidate_sensing_poses;
	double delta_angle = (plan_for_footprint == true ? 4.*PI : delta_theta);
	for(std::vector<cv::Point>::iterator center=cell_centers.begin(); center!=cell_centers.end(); ++center)
	{
		for(double angle=room_rotation_angle; angle<2.0*PI+room_rotation_angle; angle+=delta_angle)
		{
			// create and save pose
			geometry_msgs::Pose2D candidate_pose;
			candidate_pose.x = center->x;
			candidate_pose.y = center->y;
			candidate_pose.theta = angle;
			while (candidate_pose.theta < -PI)
				candidate_pose.theta += 2*PI;
			while (candidate_pose.theta > PI)
				candidate_pose.theta -= 2*PI;
			candidate_sensing_poses.push_back(candidate_pose);
		}
	}

	// ************* III. Construct the matrices needed in the linear program. *************
	// set or compute largest_robot_to_footprint_distance_pixel depending on plan_for_footprint
	double max_dist=0.;
	if (plan_for_footprint==true || largest_robot_to_footprint_distance_meter > 0.)
		max_dist = largest_robot_to_footprint_distance_meter;
	else
	{
		// find largest_robot_to_footprint_distance_pixel by checking the fov corners
		for (size_t i=0; i<fov_corners_meter.size(); ++i)
		{
			const double dist = fov_corners_meter[i].norm();
			if (dist > max_dist)
				max_dist = dist;
		}
	}
	const double largest_robot_to_footprint_distance_pixel = max_dist / map_resolution;
	const double cell_outcircle_radius_pixel = cell_size_pixel/sqrt(2);

	// construct W
	int number_of_candidates=candidate_sensing_poses.size();
	std::vector<double> W(number_of_candidates, 1.0); // initial weights

	// construct V
	cv::Mat V = cv::Mat::zeros(cell_centers.size(), number_of_candidates, CV_8U); // binary variables

	// check observable cells from each candidate pose
	const double map_resolution_inverse = 1./map_resolution;
	for(std::vector<geometry_msgs::Pose2D>::iterator pose=candidate_sensing_poses.begin(); pose!=candidate_sensing_poses.end(); ++pose)
	{
		// get the transformed field of view
		// get the rotation matrix
		const float sin_theta = std::sin(pose->theta);
		const float cos_theta = std::cos(pose->theta);
		Eigen::Matrix<float, 2, 2> R_fov;
		R_fov << cos_theta, -sin_theta, sin_theta, cos_theta;

		// transform field of view points, if the planning should be done for the field of view
		std::vector<cv::Point> transformed_fov_points;
		Eigen::Matrix<float, 2, 1> pose_as_matrix;
		if(plan_for_footprint==false)
		{
			pose_as_matrix << (pose->x*map_resolution)+map_origin.x, (pose->y*map_resolution)+map_origin.y; // convert to [meter]
			for(size_t point = 0; point < fov_corners_meter.size(); ++point)
			{
				// linear transformation
				Eigen::Matrix<float, 2, 1> transformed_vector = pose_as_matrix + R_fov * fov_corners_meter[point];

				// save the transformed point as cv::Point, also check if map borders are satisfied and transform it into pixel
				// values
				cv::Point current_point = cv::Point((transformed_vector(0, 0) - map_origin.x)*map_resolution_inverse, (transformed_vector(1, 0) - map_origin.y)*map_resolution_inverse);
				current_point.x = std::max(current_point.x, 0);
				current_point.y = std::max(current_point.y, 0);
				current_point.x = std::min(current_point.x, room_map.cols);
				current_point.y = std::min(current_point.y, room_map.rows);
				transformed_fov_points.push_back(current_point);
			}
		}

		// for each pose check the cells that are closer than the max distance from robot to fov-corner and more far away
		// than the min distance, also only check points that span an angle to the robot-to-fov vector smaller than the
		// max found angle to the corners
		// when planning for the robot footprint simply check if its distance to the pose is at most the given coverage radius
//		cv::Mat black_map = cv::Mat(room_map.rows, room_map.cols, room_map.type(), cv::Scalar(0));
//		cv::circle(black_map, cv::Point(pose->x, pose->y), 3, cv::Scalar(200), CV_FILLED);
//		cv::fillConvexPoly(black_map, transformed_fov_points, cv::Scalar(150));
		for(std::vector<cv::Point>::iterator neighbor=cell_centers.begin(); neighbor!=cell_centers.end(); ++neighbor)
		{
			// compute pose to neighbor vector
			Eigen::Matrix<float, 2, 1> pose_to_neighbor;
			pose_to_neighbor << neighbor->x-pose->x, neighbor->y-pose->y;
			double distance = pose_to_neighbor.norm();

			// if neighbor is in the possible distance range check it further, distances given in [pixel]
			if(plan_for_footprint==false && distance<=largest_robot_to_footprint_distance_pixel)
			{
//				cv::circle(black_map, *neighbor, 2, cv::Scalar(50), CV_FILLED);

				if(cv::pointPolygonTest(transformed_fov_points, *neighbor, false) >= 0) // point inside
				{
					// check if the line from the robot pose to the neighbor crosses an obstacle, if so it is not observable from the pose
					cv::LineIterator border_line(room_map, cv::Point(pose->x, pose->y), *neighbor, 8); // opencv implementation of bresenham algorithm, 8: color, irrelevant
					bool hit_obstacle = false;
					for(size_t i = 0; i < border_line.count; ++i, ++border_line)
						if(room_map.at<uchar>(border_line.pos()) == 0)
							hit_obstacle = true;

					if(hit_obstacle == false)
					{
						V.at<uchar>(neighbor-cell_centers.begin(), pose-candidate_sensing_poses.begin()) = 1;
//						cv::circle(black_map, *neighbor, 2, cv::Scalar(100), CV_FILLED);
					}
					else	// neighbor cell not observable
						V.at<uchar>(neighbor-cell_centers.begin(), pose-candidate_sensing_poses.begin()) = 0;
				}
				else	// neighbor cell outside the field of view
					V.at<uchar>(neighbor-cell_centers.begin(), pose-candidate_sensing_poses.begin()) = 0;
			}
			// check if neighbor is covered by footprint when planning for it
			// todo: check: by adding cell_outcircle_radius_pixel to the distance, we ensure that the entire cell lies within the footprint
			else if(plan_for_footprint==true && (distance+cell_outcircle_radius_pixel)<=largest_robot_to_footprint_distance_pixel)
			{
				// check if the line from the robot pose to the neighbor crosses an obstacle, if so it is not observable from the pose
				cv::LineIterator border_line(room_map, cv::Point(pose->x, pose->y), *neighbor, 8); // opencv implementation of bresenham algorithm, 8: color, irrelevant
				bool hit_obstacle = false;
				for(size_t i = 0; i < border_line.count; ++i, ++border_line)
					if(room_map.at<uchar>(border_line.pos()) == 0)
						hit_obstacle = true;
				if(hit_obstacle == false)
					V.at<uchar>(neighbor-cell_centers.begin(), pose-candidate_sensing_poses.begin()) = 1;
				else	// neighbor cell not observable
					V.at<uchar>(neighbor-cell_centers.begin(), pose-candidate_sensing_poses.begin()) = 0;
			}
			else // point not in the right range to be inside the fov
				V.at<uchar>(neighbor-cell_centers.begin(), pose-candidate_sensing_poses.begin()) = 0;
		}
//		cv::imshow("observable cells", black_map);
//		cv::waitKey();
	}
	std::cout << "number of optimization variables: " << W.size() << std::endl;

//	testing
//	for(size_t i=0; i<cell_centers.size(); ++i)
//	{
//		cv::Mat black_map = cv::Mat(room_map.rows, room_map.cols, room_map.type(), cv::Scalar(0));
//		cv::circle(black_map, cell_centers[i], 2, cv::Scalar(127), CV_FILLED);
//		for(size_t j=0; j<V.cols; ++j)
//		{
//			if(V.at<uchar>(i, j) == 1)
//			{
//				cv::circle(black_map, cv::Point(candidate_sensing_poses[j].x, candidate_sensing_poses[j].y), 2, cv::Scalar(100), CV_FILLED);
//				cv::imshow("candidates", black_map);
//				cv::waitKey();
//			}
//		}
//	}

	// ************* IV. Solve the different linear problems. *************
	// 1. solve the weighted optimization problem until a convergence in the sparsity is reached or a defined number of
	// 	  iterations is reached
	std::vector<double> C(W.size()); //initialize the objective vector
	bool sparsity_converged = false; // boolean to check, if the sparsity of C has converged to a certain value
	double weight_epsilon = 0.0; // parameter that is used to update the weights after one solution has been obtained
	uint number_of_iterations = 0;
	std::vector<uint> sparsity_measures; // vector that stores the computed sparsity measures to check convergence
	const double euler_constant = std::exp(1.0);
	Timer tim;
	do
	{
		// increase number of iterations
		++number_of_iterations;

		// solve optimization of the current step
		solveOptimizationProblem(C, V, &W);

		// update epsilon and W
		const int exponent = 1 + (number_of_iterations - 1)*0.1;
		weight_epsilon = std::pow(1./(euler_constant-1.), exponent);
		for(size_t weight=0; weight<W.size(); ++weight)
			W[weight] = weight_epsilon/(weight_epsilon + C[weight]);

		// measure sparsity of C to check terminal condition, used measure: l^0_eps (|{i: c[i] <= eps}|)
		uint sparsity_measure = 0;
		for(size_t variable=0; variable<C.size(); ++variable)
			if(C[variable]<=0.01)
				++sparsity_measure;
		sparsity_measures.push_back(sparsity_measure);

		// check terminal condition, i.e. if the sparsity hasn't improved in the last n steps using l^0_eps measure,
		// if enough iterations have been done yet
		if(sparsity_measures.size() >= sparsity_check_range)
		{
			uint number_of_last_measure = 0;
			for(std::vector<uint>::reverse_iterator measure=sparsity_measures.rbegin(); measure!=sparsity_measures.rbegin()+sparsity_check_range && measure!=sparsity_measures.rend(); ++measure)
				if(*measure >= sparsity_measures.back())
					++number_of_last_measure;

			if(number_of_last_measure == sparsity_check_range)
				sparsity_converged = true;
		}

		std::cout << "Iteration: " << number_of_iterations << ", sparsity: " << sparsity_measures.back() << std::endl;
	} while(sparsity_converged == false && number_of_iterations <= 150 && tim.getElapsedTimeInSec() < 1200);	// wait no longer than 20 minutes

	// 2. Reduce the optimization problem by discarding the candidate poses that correspond to an optimization variable
	//	  equal to 0, i.e. those that are not considered any further.
	uint new_number_of_variables = 0;
	cv::Mat V_reduced = cv::Mat(cell_centers.size(), 1, CV_8U); // initialize one column because opencv wants it this way, add other columns later
	std::vector<geometry_msgs::Pose2D> reduced_sensing_candidates;
	for(std::vector<double>::iterator result=C.begin(); result!=C.end(); ++result)
	{
		if(*result != 0.0)
		{
			// increase number of optimization variables
			++new_number_of_variables;

			// gather column corresponding to this candidate pose and add it to the new observability matrix
			cv::Mat column = V.col(result-C.begin());
			cv::hconcat(V_reduced, column, V_reduced);

			// save the new possible sensing candidate
			reduced_sensing_candidates.push_back(candidate_sensing_poses[result-C.begin()]);
		}
	}

	// remove the first initial column
	V_reduced = V_reduced.colRange(1, V_reduced.cols);

	// solve the final optimization problem
	std::cout << "new_number_of_variables=" << new_number_of_variables << std::endl;
	std::vector<int> C_reduced(new_number_of_variables);
	solveOptimizationProblem(C_reduced, V_reduced, NULL);


	// ************* V. Retrieve solution and find a path trough the chosen poses. *************
	// read out solution
	std::vector<cv::Point> chosen_positions; // vector to determine the tsp solution, in [pixels]
	std::vector<geometry_msgs::Pose2D> chosen_poses;	// in [px,px,rad]
	for(std::vector<int>::iterator result=C_reduced.begin(); result!=C_reduced.end(); ++result)
	{
		if(*result == 1)
		{
			chosen_poses.push_back(reduced_sensing_candidates[result-C_reduced.begin()]);
			chosen_positions.push_back(cv::Point(reduced_sensing_candidates[result-C_reduced.begin()].x, reduced_sensing_candidates[result-C_reduced.begin()].y));
		}
	}

//	// NOT NECESSARY, rotation back was done earlier after grid generation
//	// rotate back the chosen view points to the original map
//	room_rotation.transformPathBackToOriginalRotation(chosen_positions, chosen_poses, R);
//	for (size_t i=0; i<chosen_poses.size(); ++i)
//		chosen_positions[i] = cv::Point(chosen_poses[i].x, chosen_poses[i].y);

	if (chosen_positions.size()==0)
	{
		std::cout << "Convex SPP: chosen_positions is empty." << std::endl;
		return;
	}


	// get the distance matrix
	ROS_INFO("Constructing distance matrix");
	cv::Mat distance_matrix;
	DistanceMatrix distance_matrix_computation;
	distance_matrix_computation.constructDistanceMatrix(distance_matrix, room_map, chosen_positions, 0.25, 0.0, map_resolution, path_planner_);

	// go trough the matrix and multiply the distances between the poses with a factor corresponding to the difference of
	// the travel angle from the first pose to the other pose
	for(size_t pose=0; pose<chosen_poses.size(); ++pose)
	{
		for(size_t neighbor=0; neighbor<chosen_poses.size(); ++neighbor)
		{
			// don't look at pose to itself
			if(pose!=neighbor)
			{
				// get travel angle from current pose to next pose
				cv::Point current_point = cv::Point(chosen_poses[pose].x, chosen_poses[pose].y);
				cv::Point next_point = cv::Point(chosen_poses[neighbor].x, chosen_poses[neighbor].y);
				cv::Point vector = next_point - current_point;
				float angle = std::atan2(vector.y, vector.x);

				// abs angle distance between poses
				double delta_theta = std::abs(chosen_poses[pose].theta - angle);

				// compute penalizing function
				double penalizing_factor = 1 + (delta_theta/PI);

				// update distances
				distance_matrix.at<double>(pose, neighbor) = penalizing_factor*distance_matrix.at<double>(pose, neighbor);
				distance_matrix.at<double>(neighbor, pose) = penalizing_factor*distance_matrix.at<double>(neighbor, pose); //symmetrical-Matrix --> saves half the computation time
			}
		}
	}

	// do the repetitive nearest neighbor algorithm
	ROS_INFO("Solving TSP with repetitive nearest neighbor");
	std::vector<int> best_order;
	double min_distance = 1e9;
	for(int start=0; start<chosen_positions.size(); ++start)
	{
		// obtain nearest neighbor solution for this start
		std::vector<int> current_order = tsp_solver_.solveNearestTSP(distance_matrix, start);

		// get pathlength for this solution
		double current_pathlength = 0.0;
		for(size_t vertex=0; vertex<current_order.size(); ++vertex)
			current_pathlength += distance_matrix.at<double>(current_order[vertex], current_order[(vertex+1)%current_order.size()]);

		// if current path is better than previous one save it
		if(current_pathlength < min_distance)
		{
			min_distance = current_pathlength;
			best_order = current_order;
		}
	}

	// find the node that is closest to the starting position
	min_distance = 1e9;
	int starting_index = 0;
	for(std::vector<cv::Point>::iterator position=chosen_positions.begin(); position!=chosen_positions.end(); ++position)
	{
		// calculate current distance
		double current_distance = cv::norm(rotated_starting_position - *position);

		// check if current length is smaller than optimal
		if(current_distance < min_distance)
		{
			min_distance = current_distance;
			starting_index = position-chosen_positions.begin();
		}
	}

	// create the path starting from the found start
	std::vector<cv::Point> fov_poses;
	std::vector<int>::iterator start = std::find(best_order.begin(), best_order.end(), starting_index); // obtain iterator to index in best order to start path creation from there
	for(size_t pose=start-best_order.begin(); path.size()!=chosen_poses.size() && fov_poses.size()!=chosen_poses.size(); ++pose)
	{

		// check if end has been reached, if true start from the beginning again
		if(pose == best_order.size())
			pose = 0;

		// insert pose mapped to global coordinates
		geometry_msgs::Pose2D current_pose;
		if (plan_for_footprint == false)
		{
			// take the viewing directions as computed for fov mode, convert locations from pixels to meters
			current_pose.x = (chosen_poses[best_order[pose]].x*map_resolution)+map_origin.x;
			current_pose.y = (chosen_poses[best_order[pose]].y*map_resolution)+map_origin.y;
			current_pose.theta = chosen_poses[best_order[pose]].theta;
			path.push_back(current_pose);
		}
		else
		{
			// for footprint planning the viewing direction has to be computed from the trajectory
			fov_poses.push_back(cv::Point(chosen_poses[best_order[pose]].x, chosen_poses[best_order[pose]].y));
		}
	}

	// *********************** VI. Get the robot path out of the fov path. ***********************
	// determine the correct viewing angles for the poses (footprint planning just used one fixed dummy direction)
	if (plan_for_footprint == true)
	{
		// compute viewing directions
		room_rotation.transformPointPathToPosePath(fov_poses, path);
		// convert to meters
		for (size_t i=0; i<path.size(); ++i)
		{
			path[i].x = path[i].x * map_resolution + map_origin.x;
			path[i].y = path[i].y * map_resolution + map_origin.y;
		}
	}
//	if (plan_for_footprint == true) //todo: ?
//	{
//		// go trough all computed fov poses and compute the corresponding robot pose
//		ROS_INFO("Starting to map from field of view pose to robot pose");
//		mapPath(room_map, path, fov_poses, robot_to_fov_vector_meter, map_resolution, map_origin, starting_position);
//	}







//	testing
//	cv::Mat test_map = room_map.clone();
//	for(std::vector<geometry_msgs::Pose2D>::iterator pose=chosen_poses.begin(); pose!=chosen_poses.end(); ++pose)
//	{
//		float sin_theta = std::sin(pose->theta);
//		float cos_theta = std::cos(pose->theta);
//		Eigen::Matrix<float, 2, 2> R_fov;
//		R_fov << cos_theta, -sin_theta, sin_theta, cos_theta;
//
//		// transform field of view points
//		std::vector<cv::Point> transformed_fov_points;
//		Eigen::Matrix<float, 2, 1> pose_as_matrix;
//		pose_as_matrix << (pose->x*map_resolution)+map_origin.x, (pose->y*map_resolution)+map_origin.y; // convert to [meter]
//		for(size_t point = 0; point < field_of_view.size(); ++point)
//		{
//			// transform fov-point from geometry_msgs::Point32 to Eigen::Matrix
//			Eigen::Matrix<float, 2, 1> fov_point;
//			fov_point << field_of_view[point].x, field_of_view[point].y;
//
//			// linear transformation
//			Eigen::Matrix<float, 2, 1> transformed_vector = pose_as_matrix + R_fov * fov_point;
//
//			// save the transformed point as cv::Point, also check if map borders are satisfied and transform it into pixel
//			// values
//			cv::Point current_point = cv::Point((transformed_vector(0, 0) - map_origin.x)/map_resolution, (transformed_vector(1, 0) - map_origin.y)/map_resolution);
//			current_point.x = std::max(current_point.x, 0);
//			current_point.y = std::max(current_point.y, 0);
//			current_point.x = std::min(current_point.x, room_map.cols);
//			current_point.y = std::min(current_point.y, room_map.rows);
//			transformed_fov_points.push_back(current_point);
//		}
//
//		// rotate the vector from the robot to the field of view middle point
//		Eigen::Matrix<float, 2, 1> transformed_robot_to_middlepoint_vector = R_fov * robot_to_fov_middlepoint_vector;
//
//		cv::fillConvexPoly(test_map, transformed_fov_points, cv::Scalar(200));
//	}
//	for(size_t i=0; i<cell_centers.size(); ++i)
//		cv::circle(test_map, cell_centers[i], 2, cv::Scalar(100), CV_FILLED);
//	for(size_t i=0; i<path.size()-1; ++i)
//	{
//		cv::circle(test_map, cv::Point(path[i].x/map_resolution, path[i].y/map_resolution), 5, cv::Scalar(100), CV_FILLED);
//		cv::line(test_map, cv::Point(path[i].x/map_resolution, path[i].y/map_resolution), cv::Point(path[i+1].x/map_resolution, path[i+1].y/map_resolution), cv::Scalar(127), 2);
//	}
//	cv::circle(test_map, cv::Point(path.back().x/map_resolution, path.back().y/map_resolution), 5, cv::Scalar(100), CV_FILLED);

//	cv::imshow("seen areas", test_map);
//	cv::waitKey();
}

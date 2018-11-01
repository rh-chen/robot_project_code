#include <ros/ros.h>
#include <ros/package.h>

#include <string>
#include <vector>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <cv_bridge/cv_bridge.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Point32.h>
#include <nav_msgs/OccupancyGrid.h>

#include <ipa_building_msgs/RoomExplorationAction.h>

#include <ipa_room_exploration/dynamic_reconfigure_client.h>

#include <ipa_room_exploration/timer.h>

#include <Eigen/Dense>

// overload of << operator for geometry_msgs::Pose2D to wanted format
std::ostream& operator<<(std::ostream& os, const geometry_msgs::Pose2D& obj)
{
	std::stringstream ss;
	ss <<  "[" << obj.x << ", " << obj.y << ", " << obj.theta << "]";
	os << ss.rdbuf();
	return os;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "room_exploration_client");
	ros::NodeHandle nh;

	actionlib::SimpleActionClient<ipa_building_msgs::RoomExplorationAction> ac("room_exploration_server", true);

	// read in test map
	const std::string test_map_path = ros::package::getPath("ipa_room_segmentation") + "/common/files/test_maps/";
	const std::string map_name = test_map_path + "lab_ipa.png";
	cv::Mat map = cv::imread(map_name, 0);
	//make non-white pixels black
	for (int y = 0; y < map.rows; y++)
	{
		for (int x = 0; x < map.cols; x++)
		{
			//find not reachable regions and make them black
			if (map.at<unsigned char>(y, x) < 250)
			{
				map.at<unsigned char>(y, x) = 0;
			}
			//else make it white
			else
			{
				map.at<unsigned char>(y, x) = 255;
			}
		}
	}
	std::cout << "map-size: " << map.rows << "x" << map.cols << std::endl;

//	const std::string topic = "/move_base/global_costmap/costmap";
//	nav_msgs::OccupancyGrid grid;
//	grid = *(ros::topic::waitForMessage<nav_msgs::OccupancyGrid>(topic, nh));
//	ROS_INFO("got grid");
//
//	std::vector<signed char> dats;
//	dats = grid.data;
//
//	std::cout << dats.size() << std::endl;
//	int s = 200;
//	cv::Mat test_map = cv::Mat(s, s, map.type());
//
//	for(size_t u = 0; u < test_map.cols; ++u)
//	{
//		for(size_t v = 0; v < test_map.rows; ++v)
//		{
//			test_map.at<uchar>(u,v) = (uchar) dats[v+u*s];
//		}
//	}
//
//	cv::imshow("testtt", test_map);
//	cv::waitKey();

	ROS_INFO("Waiting for action server to start.");
	// wait for the action server to start
	ac.waitForServer(); //will wait for infinite time

	ROS_INFO("Action server started, sending goal.");

	DynamicReconfigureClient drc_exp(nh, "room_exploration_server/set_parameters", "room_exploration_server/parameter_updates");
	drc_exp.setConfig("room_exploration_algorithm", 2);
//	drc_exp.setConfig("path_eps", 3);
//	drc_exp.setConfig("grid_line_length", 15);
//	drc_exp.setConfig("path_eps", 10);
//	drc_exp.setConfig("cell_size", 10);
//	drc_exp.setConfig("plan_for_footprint", true);
//	drc_exp.setConfig("goal_eps", 0.0);
//	drc_exp.setConfig("delta_theta", 0.005);

//	cv::Point2f src_center(map.cols/2.0F, map.rows/2.0F);
//	cv::Mat rot_mat = getRotationMatrix2D(src_center, 180, 1.0);
//	cv::Mat dst;
//	cv::warpAffine(map, dst, rot_mat, map.size());
//	cv::flip(dst, map, 1);
//	cv::imshow("map", map);
//	cv::waitKey();

	sensor_msgs::Image labeling;
	cv_bridge::CvImage cv_image;
//	cv_image.header.stamp = ros::Time::now();
	cv_image.encoding = "mono8";
	cv_image.image = map;
	cv_image.toImageMsg(labeling);

	geometry_msgs::Pose map_origin;
	map_origin.position.x = 0.0;
	map_origin.position.y = 0.0;

	geometry_msgs::Pose2D starting_position;
	starting_position.x = 1.0;
	starting_position.y = 1.0;
	starting_position.theta = 0.0;

	std::vector<geometry_msgs::Point32> fov_points(4);
//	fov_points[0].x = 0.15;		// this field of view fits a Asus Xtion sensor mounted at 0.63m height (camera center) pointing downwards to the ground in a respective angle
//	fov_points[0].y = 0.35;
//	fov_points[1].x = 0.15;
//	fov_points[1].y = -0.35;
//	fov_points[2].x = 1.15;
//	fov_points[2].y = -0.65;
//	fov_points[3].x = 1.15;
//	fov_points[3].y = 0.65;
//	int planning_mode = 2;	// viewpoint planning
	fov_points[0].x = -0.3;		// this is the working area of a vacuum cleaner with 60 cm width
	fov_points[0].y = 0.3;
	fov_points[1].x = -0.3;
	fov_points[1].y = -0.3;
	fov_points[2].x = 0.3;
	fov_points[2].y = -0.3;
	fov_points[3].x = 0.3;
	fov_points[3].y = 0.3;
	int planning_mode = 1;	// footprint planning

	ipa_building_msgs::RoomExplorationGoal goal;
	goal.input_map = labeling;
	goal.map_resolution = 0.05;
	goal.map_origin = map_origin;
	goal.robot_radius = 0.3; // turtlebot, used for sim 0.177, 0.4
	goal.coverage_radius = 0.3;
	goal.field_of_view = fov_points;
	goal.starting_position = starting_position;
	goal.planning_mode = planning_mode;
	ac.sendGoal(goal);

	ac.waitForResult(ros::Duration());
	ipa_building_msgs::RoomExplorationResultConstPtr action_result = ac.getResult();

	std::cout << "Got a path with " << action_result->coverage_path.size() << " nodes." << std::endl;

	// display path
	const double inverse_map_resolution = 1./goal.map_resolution;
	cv::Mat path_map = map.clone();
	for (size_t point=0; point<action_result->coverage_path.size(); ++point)
	{
		const cv::Point point1((action_result->coverage_path[point].x-map_origin.position.x)*inverse_map_resolution, (action_result->coverage_path[point].y-map_origin.position.y)*inverse_map_resolution);
		cv::circle(path_map, point1, 2, cv::Scalar(128), -1);
		if (point > 0)
		{
			const cv::Point point2((action_result->coverage_path[point-1].x-map_origin.position.x)*inverse_map_resolution, (action_result->coverage_path[point-1].y-map_origin.position.y)*inverse_map_resolution);
			cv::line(path_map, point1, point2, cv::Scalar(128), 1);
		}
		std::cout << "coverage_path[" << point << "]: x=" << action_result->coverage_path[point].x << ", y=" << action_result->coverage_path[point].y << ", theta=" << action_result->coverage_path[point].theta << std::endl;
	}
	cv::imshow("path", path_map);
	cv::waitKey();

////	// testing
//	std::vector<cv::Point> fov(5);
//	fov[0] = cv::Point(70, 20);
//	fov[1] = cv::Point(90, 20);
//	fov[2] = cv::Point(100, 40);
//	fov[3] = cv::Point(60, 40);
//	fov[4] = cv::Point(70, 20);
//
//	cv::Point robot_pose(80, 55);
//	cv::Point test_point(80, 50);
//
//	cv::Mat white_map = cv::Mat(200, 200, CV_8U, cv::Scalar(255));
////	cv::line(white_map, cv::Point(80, 20), cv::Point(90, 20), cv::Scalar(0), 1);
////	cv::line(white_map, cv::Point(90, 20), cv::Point(75, 25), cv::Scalar(0), 1);
//
//	Timer tim;
//	tim.start();
////	std::cout << pointInsidePolygonCheck(robot_pose, fov) << std::endl;
////	std::cout << pointInsidePolygonCheck(test_point, fov) << std::endl;
//
//	// find points that span biggest angle
//	std::vector<Eigen::Matrix<double, 2, 1> > fov_vectors;
//	for(int i = 0; i < 4; ++i)
//	{
//		Eigen::Matrix<double, 2, 1> current_vector;
//		current_vector << fov[i].x - robot_pose.x , fov[i].y - robot_pose.y;
//
//		fov_vectors.push_back(current_vector);
//	}
//
//	// get angles
//	double dot = fov_vectors[0].transpose()*fov_vectors[1];
//	double abs = fov_vectors[0].norm() * fov_vectors[1].norm();
//	double angle_1 = std::acos(dot/abs);
//	dot = fov_vectors[2].transpose()*fov_vectors[3];
//	abs = fov_vectors[2].norm() * fov_vectors[3].norm();
//	double angle_2 = std::acos(dot/abs);
//
//	// get points that define the edge-points of the line the raycasting should go to, by computing the intersection of two
//	// lines: the line defined by the robot pose and the fov-point that spans the highest angle and a line parallel to the
//	// front side of the fov with an offset
//	Eigen::Matrix<double, 2, 1> end_point_1, end_point_2, robot_pose_as_vector;
//	robot_pose_as_vector << robot_pose.x, robot_pose.y;
//	double border_distance = 5;
//	Eigen::Matrix<double, 2, 1> pose_to_fov_edge_vector_1 = fov_vectors[0];
//	Eigen::Matrix<double, 2, 1> pose_to_fov_edge_vector_2 = fov_vectors[1];
//	if(angle_1 > angle_2) // do line crossings s.t. the corners are guaranteed to be after the fov
//	{
//		// get vectors showing the directions for for the lines from pose to edge of fov
//		Eigen::Matrix<double, 2, 1> normed_fov_vector_1 = fov_vectors[0]/fov_vectors[0].norm();
//		Eigen::Matrix<double, 2, 1> normed_fov_vector_2 = fov_vectors[1]/fov_vectors[1].norm();
//
//		// get the offset point after the end of the fov
//		Eigen::Matrix<double, 2, 1> offset_point_after_fov = fov_vectors[2];
//		offset_point_after_fov(1, 0) = offset_point_after_fov(1, 0) + border_distance;
//
//		// find the parameters for the two different intersections (for each corner point)
//		double first_edge_parameter = (pose_to_fov_edge_vector_1(1, 0)/pose_to_fov_edge_vector_1(0, 0) * (fov_vectors[0](0, 0) - offset_point_after_fov(0, 0)) + offset_point_after_fov(1, 0) - fov_vectors[0](1, 0))/( pose_to_fov_edge_vector_1(1, 0)/pose_to_fov_edge_vector_1(0, 0) * (fov_vectors[3](0, 0) - fov_vectors[2](0, 0)) - (fov_vectors[3](1, 0) - fov_vectors[2](1, 0)) );
//		double second_edge_parameter = (pose_to_fov_edge_vector_2(1, 0)/pose_to_fov_edge_vector_2(0, 0) * (fov_vectors[1](0, 0) - offset_point_after_fov(0, 0)) + offset_point_after_fov(1, 0) - fov_vectors[1](1, 0))/( pose_to_fov_edge_vector_2(1, 0)/pose_to_fov_edge_vector_2(0, 0) * (fov_vectors[3](0, 0) - fov_vectors[2](0, 0)) - (fov_vectors[3](1, 0) - fov_vectors[2](1, 0)) );
//
//		// use line equations and found parameters to actually find the corners
//		end_point_1 = first_edge_parameter * (fov_vectors[3] - fov_vectors[2]) + offset_point_after_fov + robot_pose_as_vector;
//		end_point_2 = second_edge_parameter * (fov_vectors[3] - fov_vectors[2]) + offset_point_after_fov + robot_pose_as_vector;
//	}
//	else
//	{
//		// follow the lines to the farthest points and go a little longer, this ensures that the whole fov is covered
//		double travel_distance = 1.2 * fov_vectors[2].norm(); // from current pose to most far points
//		end_point_1 = 2.0 * fov_vectors[2] + robot_pose_as_vector;
//		end_point_2 = 2.0 * fov_vectors[3] + robot_pose_as_vector;
//	}
//
//	Eigen::Matrix<double, 2, 1> g = 4.0 * pose_to_fov_edge_vector_1 + fov_vectors[0] + robot_pose_as_vector;
////	cv::line(white_map, robot_pose, cv::Point(g(0,0), g(1,0)), cv::Scalar(120), 1);
//	g = 4.0 * pose_to_fov_edge_vector_2 + fov_vectors[1] + robot_pose_as_vector;
////	cv::line(white_map, robot_pose, cv::Point(g(0,0), g(1,0)), cv::Scalar(120), 1);
//
//	// transform to OpenCv format
//	cv::Point corner_1 (end_point_1(0, 0), end_point_1(1, 0));
//	cv::Point corner_2 (end_point_2(0, 0), end_point_2(1, 0));
//
//	std::cout << "corners: " << std::endl << corner_1 << std::endl << corner_2 << std::endl;
//
//	// get points between the edge-points to get goals for raycasting
//	cv::LineIterator border_line(white_map, corner_1, corner_2, 8); // opencv implementation of bresenham algorithm, 8: color, irrelevant
//	std::vector<cv::Point> raycasting_goals(border_line.count);
//
//	for(size_t i = 0; i < border_line.count; i++, ++border_line)
//		raycasting_goals[i] = border_line.pos();
//
////	for(size_t i = 0; i < raycasting_goals.size(); ++i)
////	{
////		white_map.at<uchar>(raycasting_goals[i]) = 127;
////	}
//
//	cv::circle(white_map, corner_1, 2, cv::Scalar(100), CV_FILLED);
//	cv::circle(white_map, corner_2, 2, cv::Scalar(100), CV_FILLED);
//	cv::fillConvexPoly(white_map, fov, cv::Scalar(200));
//
//	// go trough the found raycasting goals and draw the field-of-view
//	for(std::vector<cv::Point>::iterator goal = raycasting_goals.begin(); goal != raycasting_goals.end(); ++goal)
//	{
//		// use openCVs bresenham algorithm to find the points from the robot pose to the goal
//		cv::LineIterator ray_points(white_map, robot_pose, *goal, 8);
//
//		// go trough the points on the ray and draw them if they are inside the fov, stop the current for-step when a black
//		// pixel is hit (an obstacle stops the camera from seeing whats behind)
//		for(size_t point = 0; point < ray_points.count; point++, ++ray_points)
//		{
//			if(white_map.at<unsigned char>(ray_points.pos()) == 0)
//			{
//				break;
//			}
//
//			if (white_map.at<unsigned char>(ray_points.pos()) > 0 && pointInsidePolygonCheck(ray_points.pos(), fov) == 1)
////			if (white_map.at<unsigned char>(ray_points.pos()) > 0 && cv::pointPolygonTest(fov, ray_points.pos(), false) >= 0)
//			{
//				white_map.at<uchar> (ray_points.pos()) = 127;
//			}
//		}
//	}
//
//	std::cout << std::endl << "ellapsed time: " << tim.getElapsedTimeInMilliSec() << "ms" << std::endl;
//
////	for (double distance = 1; distance < 50; ++distance)
////	{
////		const int ny = robot_pose.y + simulated_sin * distance;
////		const int nx = robot_pose.x + simulated_cos * distance;
////		//make sure the simulated point isn't out of the boundaries of the map
////		if (ny < 0 || ny >= map.rows || nx < 0 || nx >= map.cols)
////			break;
////		if (white_map.at<unsigned char>(ny, nx) > 0 && pointInsidePolygonCheck(cv::Point(nx, ny), fov) == 1)
////		{
////			white_map.at<uchar> (ny, nx) = 127;
////		}
////	}
//
//	cv::circle(white_map, robot_pose, 2, cv::Scalar(100), CV_FILLED);
//
//	cv::namedWindow("seen area", cv::WINDOW_NORMAL);
//	cv::imshow("seen area", white_map);
//	cv::resizeWindow("seen area", 600, 600);
//	cv::waitKey();

	return 0;
}

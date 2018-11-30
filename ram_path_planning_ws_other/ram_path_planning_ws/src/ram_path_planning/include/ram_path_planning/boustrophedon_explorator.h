#ifndef BE_H_
#define BE_H_

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/matx.hpp>
#include <iostream>
#include <vector>
#include <cmath>
#include <string>

#include <Eigen/Dense>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Point32.h>
#include <climits>
#include <ram_path_planning/A_star_pathplanner.h>
#define PI 3.14159265359



// Class that is used to store cells and obstacles in a certain manner. For this the vertexes are stored as points and
// the edges are stored as vectors in a counter-clockwise manner. The constructor becomes a set of respectively sorted
// points and computes the vectors out of them. Additionally the visible center of the polygon gets computed, to
// simplify the visiting order later, by using a meanshift algorithm.
class GeneralizedPolygon
{
	public:
	// vertexes
	std::vector<cv::Point> vertices_;

	// center
	cv::Point center_;

	// min/max coordinates
	int max_x_, min_x_, max_y_, min_y_;

	// constructor
	GeneralizedPolygon(const std::vector<cv::Point>& vertices, const double map_resolution):
	max_x_(0),min_x_(65536),max_y_(0),min_y_(65536)
	{
		// save given vertexes
		vertices_ = vertices;

		for(size_t point=0; point<vertices_.size(); ++point)
		{
			if(vertices_[point].x > max_x_)
				max_x_ = vertices_[point].x;
			if(vertices_[point].y > max_y_)
				max_y_ = vertices_[point].y;
			if(vertices_[point].x < min_x_)
				min_x_ = vertices_[point].x;
			if(vertices_[point].y < min_y_)
				min_y_ = vertices_[point].y;
		}
		
		double center_x = 0;
		double center_y = 0;

		for(size_t point = 0;point < vertices_.size();point++){
			center_x += vertices_[point].x;
			center_y += vertices_[point].y;
		}

		center_.x = center_x/(vertices_.size()*1.0);
		center_.y = center_y/(vertices_.size()*1.0);
	}

	cv::Point getCenter() const
	{
		return center_;
	}

	std::vector<cv::Point> getVertices() const
	{
		return vertices_;
	}

	void getMinMaxCoordinates(int& min_x, int& max_x, int& min_y, int& max_y)
	{
		min_x = min_x_;
		max_x = max_x_;
		min_y = min_y_;
		max_y = max_y_;
	}
	void drawPolygon(cv::Mat& image, const cv::Scalar& color) const
	{
		// draw polygon in an black image with necessary size
		cv::Mat black_image = cv::Mat(max_y_+10, max_x_+10, CV_8UC1, cv::Scalar(0));
		cv::drawContours(black_image, std::vector<std::vector<cv::Point> >(1,vertices_), -1, color, CV_FILLED);
		image = black_image.clone();
	}
};

// Structure to save edges of a path on one row, that allows to easily get the order of the edges when planning the
// boustrophedon path.
struct BoustrophedonHorizontalLine
{
	cv::Point left_corner_, right_corner_;
};


// Class that generates a room exploration path by using the morse cellular decomposition method, proposed by
//
// "H. Choset, E. Acar, A. A. Rizzi and J. Luntz,
// "Exact cellular decompositions in terms of critical points of Morse functions," Robotics and Automation, 2000. Proceedings.
// ICRA '00. IEEE International Conference on, San Francisco, CA, 2000, pp. 2270-2277 vol.3."
//
// This method decomposes the given environment into several cells which don't have any obstacle in it. For each of this
// cell the boustrophedon path is generated, which goes up and down in each cell, see the upper paper for reference.
// This class only produces a static path, regarding the given map in form of a point series. To react on dynamic
// obstacles, one has to do this in upper algorithms.
//
class BoustrophedonExplorer
{
protected:
	AStarPlanner path_planner_;

	// rotates the original map for a good axis alignment and divides it into Morse cells
	// the functions tries two axis alignments with 90deg rotation difference and chooses the one with the lower number of cells
	void findBestCellDecomposition(const cv::Mat& room_map, const float map_resolution, const double min_cell_area,
			cv::Mat& R, cv::Rect& bbox, cv::Mat& rotated_room_map,
			std::vector<GeneralizedPolygon>& cell_polygons, std::vector<cv::Point>& polygon_centers);

	// rotates the original map for a good axis alignment and divides it into Morse cells
	// @param rotation_offset can be used to put an offset to the computed rotation for good axis alignment, in [rad]
	void computeCellDecompositionWithRotation(const cv::Mat& room_map, const float map_resolution, const double min_cell_area,
			const double rotation_offset, cv::Mat& R, cv::Rect& bbox, cv::Mat& rotated_room_map,
			std::vector<GeneralizedPolygon>& cell_polygons, std::vector<cv::Point>& polygon_centers);

	// divides the provided map into Morse cells
	void computeCellDecomposition(const cv::Mat& room_map, const float map_resolution, const double min_cell_area,
			std::vector<GeneralizedPolygon>& cell_polygons, std::vector<cv::Point>& polygon_centers);

	// this function corrects obstacles that are one pixel width at 45deg angle, i.e. a 2x2 pixel neighborhood with [0, 255, 255, 0] or [255, 0, 0, 255]
	void correctThinWalls(cv::Mat& room_map);

	// computes the Boustrophedon path pattern for a single cell
	void computeBoustrophedonPath(const cv::Mat& room_map, const float map_resolution, const GeneralizedPolygon& cell,
			std::vector<cv::Point>& fov_middlepoint_path, cv::Point& robot_pos,
			const int grid_spacing_as_int, const int half_grid_spacing_as_int, const double path_eps);

	// downsamples a given path original_path to waypoint distances of path_eps and appends the resulting path to downsampled_path
	void downsamplePath(const std::vector<cv::Point>& original_path, std::vector<cv::Point>& downsampled_path,
			cv::Point& cell_robot_pos, const double path_eps);

	// downsamples a given path original_path to waypoint distances of path_eps in reverse order as given in original_path
	// and appends the resulting path to downsampled_path
	void downsamplePathReverse(const std::vector<cv::Point>& original_path, std::vector<cv::Point>& downsampled_path,
			cv::Point& robot_pos, const double path_eps);

public:
	// constructor
	BoustrophedonExplorer();

	// Function that creates an exploration path for a given room. The room has to be drawn in a cv::Mat (filled with Bit-uchar),
	// with free space drawn white (255) and obstacles as black (0). It returns a series of 2D poses that show to which positions
	// the robot should drive at.
	void getExplorationPath(const cv::Mat& room_map, std::vector<geometry_msgs::Pose2D>& path, const float map_resolution,
				const cv::Point starting_position, const cv::Point2d map_origin,
				const double grid_spacing_in_pixel, const double path_eps, const bool plan_for_footprint,
				const Eigen::Matrix<float, 2, 1> robot_to_fov_vector, const double min_cell_area);
};
#endif

#include <string>
#include <strings.h>
#include <eigen_conversions/eigen_msg.h>
#include <ram_path_planning/donghong_ding.hpp>
#include <ram_path_planning/AdditiveManufacturingTrajectory.h>
#include <ram_path_planning/cgutil.hpp>
#include <ram_path_planning/cpp_uav.hpp>
#include <ros/ros.h>
#include <unique_id/unique_id.h>
#include <visualization_msgs/Marker.h>
#include <ram_path_planning/Cpp.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/matx.hpp>
#include <deque>
#include <polypartition.h>
#include <boost/shared_ptr.hpp>
#include <map>
#include <algorithm>
#include <CGAL/create_offset_polygons_2.h>
#include <stdio.h>
#include <stdlib.h>
#include <queue>
#include <stack>
#include <array>
#include <numeric>
#include <exception>
#include <functional>


typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef Kernel::FT Ft;
typedef Kernel::Point_2 PointCgal;
typedef Kernel::Segment_2 SegmentCgal;
typedef Kernel::Direction_2 DirectionCgal;
typedef Kernel::Line_2 LineCgal;
typedef Kernel::Vector_2 VectorCgal;
typedef CGAL::Polygon_2<Kernel> PolygonCgal;
typedef CGAL::Polygon_with_holes_2<Kernel> PolygonWithHolesCgal;
typedef std::list<PolygonCgal> PolygonListCgal;
typedef std::vector<PointCgal> ContainerCgal;
typedef CGAL::Straight_skeleton_2<K>  Ss;
typedef boost::shared_ptr<Ss> SsPtr;
typedef CGAL::Straight_skeleton_2<K>::Vertex_const_handle     Vertex_const_handle ;
typedef CGAL::Straight_skeleton_2<K>::Halfedge_const_handle   Halfedge_const_handle ;
typedef boost::shared_ptr<PolygonCgal> PolygonPtr;
typedef std::vector<PolygonPtr> PolygonPtrVector;

namespace Cpp{
#if 1
	/** @brief k-d tree class.
	*/
	template <class PointT>
	class KDTree
	{
	public:
		/** @brief The constructors.
		*/
		KDTree() : root_(nullptr) {};
		KDTree(const std::vector<PointT>& points) : root_(nullptr) { build(points); }

		/** @brief The destructor.
		*/
		~KDTree() { clear(); }

		/** @brief Re-builds k-d tree.
		*/
		void build(const std::vector<PointT>& points)
		{
			clear();

			points_ = points;

			std::vector<int> indices(points.size());
			std::iota(std::begin(indices), std::end(indices), 0);

			root_ = buildRecursive(indices.data(), (int)points.size(), 0);
		}

		/** @brief Clears k-d tree.
		*/
		void clear()
		{ 
			clearRecursive(root_);
			root_ = nullptr;
			points_.clear();
		}

		/** @brief Validates k-d tree.
		*/
		bool validate() const
		{
			try
			{
				validateRecursive(root_, 0);
			}
			catch (const Exception&)
			{
				return false;
			}

			return true;
		}

		/** @brief Searches the nearest neighbor.
		*/
		int nnSearch(const PointT& query, double* minDist = nullptr) const
		{
			int guess;
			double _minDist = std::numeric_limits<double>::max();

			nnSearchRecursive(query, root_, &guess, &_minDist);

			if (minDist)
				*minDist = _minDist;

			return guess;
		}

		/** @brief Searches k-nearest neighbors.
		*/
		std::vector<int> knnSearch(const PointT& query, int k) const
		{
			KnnQueue queue(k);
			knnSearchRecursive(query, root_, queue, k);
			
			std::vector<int> indices(queue.size());
			for (size_t i = 0; i < queue.size(); i++)
				indices[i] = queue[i].second;

			return indices;
		}

		/** @brief Searches neighbors within radius.
		*/
		std::vector<int> radiusSearch(const PointT& query, double radius) const
		{
			std::vector<int> indices;
			radiusSearchRecursive(query, root_, indices, radius);
			return indices;
		}

	private:

		/** @brief k-d tree node.
		*/
		struct Node
		{
			int idx;       //!< index to the original point
			Node* next[2]; //!< pointers to the child nodes
			int axis;      //!< dimension's axis

			Node() : idx(-1), axis(-1) { next[0] = next[1] = nullptr; }
		};

		/** @brief k-d tree exception.
		*/
		class Exception : public std::exception { using std::exception::exception; };

		/** @brief Bounded priority queue.
		*/
		template <class T, class Compare = std::less<T>>
		class BoundedPriorityQueue
		{
		public:

			BoundedPriorityQueue() = delete;
			BoundedPriorityQueue(size_t bound) : bound_(bound) { elements_.reserve(bound + 1); };

			void push(const T& val)
			{
				auto it = std::find_if(std::begin(elements_), std::end(elements_),
					[&](const T& element){ return Compare()(val, element); });
				elements_.insert(it, val);

				if (elements_.size() > bound_)
					elements_.resize(bound_);
			}

			const T& back() const { return elements_.back(); };
			const T& operator[](size_t index) const { return elements_[index]; }
			size_t size() const { return elements_.size(); }

		private:
			size_t bound_;
			std::vector<T> elements_;
		};

		/** @brief Priority queue of <distance, index> pair.
		*/
		using KnnQueue = BoundedPriorityQueue<std::pair<double, int>>;

		/** @brief Builds k-d tree recursively.
		*/
		Node* buildRecursive(int* indices, int npoints, int depth)
		{
			if (npoints <= 0)
				return nullptr;

			const int axis = depth % PointT::DIM;
			const int mid = (npoints - 1) / 2;

			std::nth_element(indices, indices + mid, indices + npoints, [&](int lhs, int rhs)
			{
				return points_[lhs][axis] < points_[rhs][axis];
			});

			Node* node = new Node();
			node->idx = indices[mid];
			node->axis = axis;

			node->next[0] = buildRecursive(indices, mid, depth + 1);
			node->next[1] = buildRecursive(indices + mid + 1, npoints - mid - 1, depth + 1);

			return node;
		}

		/** @brief Clears k-d tree recursively.
		*/
		void clearRecursive(Node* node)
		{
			if (node == nullptr)
				return;

			if (node->next[0])
				clearRecursive(node->next[0]);

			if (node->next[1])
				clearRecursive(node->next[1]);

			delete node;
		}

		/** @brief Validates k-d tree recursively.
		*/
		void validateRecursive(const Node* node, int depth) const
		{
			if (node == nullptr)
				return;

			const int axis = node->axis;
			const Node* node0 = node->next[0];
			const Node* node1 = node->next[1];

			if (node0 && node1)
			{
				if (points_[node->idx][axis] < points_[node0->idx][axis])
					throw Exception();

				if (points_[node->idx][axis] > points_[node1->idx][axis])
					throw Exception();
			}

			if (node0)
				validateRecursive(node0, depth + 1);

			if (node1)
				validateRecursive(node1, depth + 1);
		}

		static double distance(const PointT& p, const PointT& q)
		{
			double dist = 0;
			for (size_t i = 0; i < PointT::DIM; i++)
				dist += (p[i] - q[i]) * (p[i] - q[i]);
			return sqrt(dist);
		}

		/** @brief Searches the nearest neighbor recursively.
		*/
		void nnSearchRecursive(const PointT& query, const Node* node, int *guess, double *minDist) const
		{
			if (node == nullptr)
				return;

			const PointT& train = points_[node->idx];

			const double dist = distance(query, train);
			if (dist < *minDist)
			{
				*minDist = dist;
				*guess = node->idx;
			}

			const int axis = node->axis;
			const int dir = query[axis] < train[axis] ? 0 : 1;
			nnSearchRecursive(query, node->next[dir], guess, minDist);

			const double diff = fabs(query[axis] - train[axis]);
			if (diff < *minDist)
				nnSearchRecursive(query, node->next[!dir], guess, minDist);
		}

		/** @brief Searches k-nearest neighbors recursively.
		*/
		void knnSearchRecursive(const PointT& query, const Node* node, KnnQueue& queue, int k) const
		{
			if (node == nullptr)
				return;

			const PointT& train = points_[node->idx];

			const double dist = distance(query, train);
			queue.push(std::make_pair(dist, node->idx));

			const int axis = node->axis;
			const int dir = query[axis] < train[axis] ? 0 : 1;
			knnSearchRecursive(query, node->next[dir], queue, k);

			const double diff = fabs(query[axis] - train[axis]);
			if ((int)queue.size() < k || diff < queue.back().first)
				knnSearchRecursive(query, node->next[!dir], queue, k);
		}

		/** @brief Searches neighbors within radius.
		*/
		void radiusSearchRecursive(const PointT& query, const Node* node, std::vector<int>& indices, double radius) const
		{
			if (node == nullptr)
				return;

			const PointT& train = points_[node->idx];

			const double dist = distance(query, train);
			if (dist < radius)
				indices.push_back(node->idx);

			const int axis = node->axis;
			const int dir = query[axis] < train[axis] ? 0 : 1;
			radiusSearchRecursive(query, node->next[dir], indices, radius);

			const double diff = fabs(query[axis] - train[axis]);
			if (diff < radius)
				radiusSearchRecursive(query, node->next[!dir], indices, radius);
		}

		Node* root_;                 //!< root node
		std::vector<PointT> points_; //!< points
	};
#endif
#if 1
//#define M 1024

typedef struct node_t
{
    cv::Point2f* polygon; 
    int n_children;
    int n_point;
    int level;              
    struct node_t** children;

    node_t(int n_point_,int n_children_,int l_){
        polygon = new cv::Point2f[n_point_];
        n_point = n_point_;
        n_children = n_children_;
        level = l_;
        children = NULL;
    }
} NODE;

void BfsTree(NODE* head){
    std::vector<NODE*> vec;
    std::queue<NODE*> q;
    NODE* p;
    
    std::stack<NODE*> s;

    q.push(head);
    s.push(head);
    while(!q.empty()){
        p = q.front();
        q.pop();
     
        ROS_INFO_STREAM("ptr:" << p);
        for(int i = 0;i < p->n_children;i++){
            q.push(p->children[i]);
            ROS_INFO_STREAM("ptr_:" << p->children[i]);
            s.push(p->children[i]);
        }
        ROS_INFO_STREAM("*************************************************");
    }

    while(!s.empty()){
        p = s.top();
        s.pop();
        
        delete[] p->polygon;
        delete[] p->children;
    }
}
#endif

#if 1
cv::Point2i getTopLeftPoint(cv::Mat& image) {
  int nRows = image.rows;
  int nCols = image.cols;

  if (image.isContinuous()) {
    nCols *= nRows;
    nRows = 1;
  }

  unsigned char* p;

  for (int i = 0; i < nRows; ++i) {
    p = image.ptr(i);
    for (int j = 0; j < nCols; ++j) {
      if (p[j] == 255) {
        if (image.isContinuous()) {
          nCols = image.cols;
          cv::Point2i P(j % nCols, j / nCols);
          return P;
        } else {
          cv::Point2i P(j, i);
          return P;
        }
      }
    }
  }

  cv::Point2i P(-1, -1);
  return P;
}

cv::Point2i getStartPoint(cv::Mat& img, cv::Point2i p, int gsize) {
  int qx, qy;
  qx = (ceil(float(p.x) / gsize) - 1) * gsize;
  qy = (ceil(float(p.y) / gsize) - 1) * gsize;
  cv::Point2i q(qx, qy);
  return q;
}

bool objectInUGB(cv::Mat& img, cv::Point2i q, int ugb, int gsize) {
  cv::Point2i pt;
  switch (ugb) {
    case 1:
      pt.x = q.x;
      pt.y = q.y - gsize;
      break;
    case 2:
      pt.x = q.x - gsize;
      pt.y = q.y - gsize;
      break;
    case 3:
      pt.x = q.x - gsize;
      pt.y = q.y;
      break;
    case 4:
      pt.x = q.x;
      pt.y = q.y;
      break;
    default:
      break;
  }

  if (pt.x < 0 || pt.y < 0 || pt.x >= img.cols || pt.y >= img.rows) {
    return false;
  }

  unsigned char* p;
  for (int i = pt.y; i <= pt.y + gsize; i++) {
    p = img.ptr(i);
    for (int j = pt.x; j <= pt.x + gsize; ++j) {
      if (p[j] == 255) {
        return true;
      }
    }
  }
  return false;
}

int getPointType(cv::Mat& img, cv::Point2i q, int gsize) {
  int m = 0, r = 0, t = 10;
  for (int k = 1; k < 5; k++) {
    if (objectInUGB(img, q, k, gsize)) {
      m++;
      r += k;
    }
  }
  if (m == 2 && (r == 4 || r == 6)) {
    t = -2;
  } else if (m == 0 || m == 4) {
    t = 0;
  } else {
    t = 2 - m;
  }
  return t;
}

cv::Point2i getNextPoint(cv::Point2i currentpoint, int d, int gsize) {
  cv::Point2i nextpoint;
  switch (d) {
    case 0:
      nextpoint.x = currentpoint.x + gsize;
      nextpoint.y = currentpoint.y;
      break;
    case 1:
      nextpoint.x = currentpoint.x;
      nextpoint.y = currentpoint.y - gsize;
      break;
    case 2:
      nextpoint.x = currentpoint.x - gsize;
      nextpoint.y = currentpoint.y;
      break;
    case 3:
      nextpoint.x = currentpoint.x;
      nextpoint.y = currentpoint.y + gsize;
      break;
  }
  return nextpoint;
}

std::vector<cv::Point2i> makeOIP(cv::Mat& img, int gsize) {
  std::vector<cv::Point2i> vertices;

  cv::Point2i topleftpoint = getTopLeftPoint(img);
  //ROS_INFO("topleftpoint:%d,%d",topleftpoint.x,topleftpoint.y);
  cv::Point2i startpoint = getStartPoint(img, topleftpoint, gsize);
  //ROS_INFO("startpoint:%d,%d",startpoint.x,startpoint.y);
  cv::Point2i q = startpoint;
  int type = getPointType(img, q, gsize);

  int d = (2 + type) % 4;
  do {
    if (type == 1 || type == -1) {
      vertices.push_back(q);
    }
    q = getNextPoint(q, d, gsize);
    type = getPointType(img, q, gsize);
    if (type == -2) {
      type = -1;
    }
    d = (d + type) % 4;
    if (d < 0) {
      d += 4;
    }
  } while (q != startpoint);

  return vertices;
}

#endif
/*bool hasConvexDefects(std::vector<cv::Vec4i>& defects_,int start_,int end_,int& mid_){
	for(int i = 0;i < defects_.size();i++){
		if((defects_[i][0] == start_) && (defects_[i][1] == end_)){
			if(defects_[i][3]/256 > DEFECT_LIMIT){
				mid_ = defects_[i][2];
				return true;
			}
		}
	}
	return false;
}*/

double polygonArea(PointVector& pv_,int n){
	if(n < 3)
		return 0;

	double sum = pv_[0].y * (pv_[n-1].x - pv_[1].x);
	for(int i = 1;i < n;i++)
		sum += pv_[i].y * (pv_[i-1].x - pv_[(i+1) % n].x);
	
	sum *= 0.5;

	return sum;
}
bool compare_polygon(const PolygonCgal& poly_a,const PolygonCgal& poly_b){
    
    double a_x = 0;
    double a_y = 0;
    double b_x = 0;
    double b_y = 0;

    for(int j = 0;j < poly_a.size();j++){
        PointCgal p = poly_a.vertex(j);

        a_x += p.x();
        a_y += p.y();
    }

    a_x /= (poly_a.size()*1.0);
    a_y /= (poly_a.size()*1.0);

    for(int j = 0;j < poly_b.size();j++){
        PointCgal p = poly_b.vertex(j);

        b_x += p.x();
        b_y += p.y();
    }

    b_x /= (poly_b.size()*1.0);
    b_y /= (poly_b.size()*1.0);


    if(a_x < b_x)
        return true;
    else{
        if(fabs(a_x-b_x) < 1e-6)
        {
            if(a_y > b_y)
                return true;
            else
                return false;
        }
        else
            return false;
    }
}

/*bool isBadPolygon(PointVector& polygon_,double step_){
    
    if(polygon_.size() < 3)
    {
        return true;
    }

    PointVector polygon_convex_hull;
    Direction sweepDirection = identifyOptimalSweepDir(polygon_,polygon_convex_hull);

    double rotationAngle = calculateHorizontalAngle(sweepDirection.baseEdge.front(), sweepDirection.baseEdge.back());
    PointVector rotatedPolygon = rotatePoints(polygon_convex_hull, -rotationAngle);
	
    for(int i = 0;i < rotatedPolygon.size();i++){
		ROS_INFO("zig_rotatedPolygon_X:%f,zig_rotatedPolygon_Y:%f",rotatedPolygon[i].x,rotatedPolygon[i].y);
	}
    
    PointVector dir{ sweepDirection.opposedVertex, sweepDirection.baseEdge.front(), sweepDirection.baseEdge.back() };
    dir = rotatePoints(dir, -rotationAngle);
    Direction rotatedDir;
    rotatedDir.opposedVertex = dir.at(0);
    rotatedDir.baseEdge.front() = dir.at(1);
    rotatedDir.baseEdge.back() = dir.at(2);

    ROS_INFO("zig_rotatedDir.baseEdge.at(0).y:%f",rotatedDir.baseEdge.at(0).y);
    ROS_INFO("zig_rotatedDir.baseEdge.at(0).x:%f",rotatedDir.baseEdge.at(0).x);
    ROS_INFO("zig_rotatedDir.baseEdge.at(1).y:%f",rotatedDir.baseEdge.at(1).y);
    ROS_INFO("zig_rotatedDir.baseEdge.at(1).x:%f",rotatedDir.baseEdge.at(1).x);

    double verticalDistance = calculateDistance(rotatedDir.baseEdge, rotatedDir.opposedVertex);
    ROS_INFO_STREAM("zig_verticalDistance:" << verticalDistance);

    if(verticalDistance < step_)
            return true;
    else{
        if(fabs(polygonArea(polygon_convex_hull,polygon_convex_hull.size())) < 0.4)
            return true;
        else
            return false;
    }
}*/

/*void selectPolygon(PolygonListCgal& src_,PolygonListCgal& dst_)
{
    std::list<PolygonCgal>::iterator iter;
    for(iter = src_.begin();iter != src_.end();iter++){
        PointVector polygon_bcd;

        PolygonCgal poly_cell = *iter;
        for(int j = 0;j < poly_cell.size();j++){
            geometry_msgs::Point point_divide_bcd;

            PointCgal p = poly_cell.vertex(j);

            point_divide_bcd.x = p.x();
            point_divide_bcd.y = p.y();
                
            polygon_bcd.push_back(point_divide_bcd);
       }

       if(!isBadPolygon(polygon_bcd,0.3))
             dst_.push_back(*iter);
    }

}*/

typedef struct SS_Edge{
    cv::Point from;
    bool isSplitNodeFrom;
    cv::Point to;
    bool isSplitNodeTo;
    double time;

    SS_Edge(cv::Point& from_,cv::Point& to_,bool is_from,bool is_to){
        from.x = from_.x;
        from.y = from_.y;
        to.x = to_.x;
        to.x = to_.y;

        isSplitNodeFrom = is_from;
        isSplitNodeTo = is_to;
        time = 0.f;
    }

    bool operator < (const SS_Edge& p)const
    {
        if((from.x*from.y*to.x*to.y) == (p.from.y*p.from.x*p.to.x*p.to.y)){
            if((from.x+from.y+to.x+to.y) == (p.from.y+p.from.x+p.to.x+p.to.y)){
                return (from.x+from.y) < (p.from.x+p.from.y)?true:false;
            }
            else
                return (from.x+from.y+to.x+to.y) < (p.to.x+p.to.y+p.from.x+p.from.y)?true:false;
        }
        else
            return (from.x*from.y*to.x*to.y) < (p.from.x*p.from.y*p.to.x*p.to.y)?true:false;
    }

}SSEdge;

typedef struct Split_Edge{
    int start_id;
    int end_id;

    Split_Edge(int start_id_,int end_id_){
        start_id = start_id_;
        end_id = end_id_;
    }

    /*bool operator < (const Split_Edge& e)const
    {
        if((start_id*end_id) == (e.start_id*e.end_id))
            return ((start_id+end_id) < (e.start_id+e.end_id)?true:false);

        else
            return ((start_id*end_id) < (e.start_id*e.end_id)?true:false);
    }*/
}SPEdge;

class edgeCmp{
    public:
    bool operator()(SPEdge const &e_a,SPEdge const &e_b)const
    {
        if((e_a.start_id*e_a.end_id) == (e_b.start_id*e_b.end_id)){
            return (e_a.start_id < e_b.start_id?true:false);
        }
        else
            return ((e_a.start_id*e_a.end_id) < (e_b.start_id*e_b.end_id)?true:false);

    }
};

void PolygonCgalToPtr(cv::Point2f* ptr,int n,PolygonCgal& poly){
    for(int i = 0;i < n;i++){
        poly.push_back(PointCgal(ptr[i].x,ptr[i].y));
    }
}

class LocalPoint : public std::array<double, 2>
{
public:

	// dimension of space (or "k" of k-d tree)
	// KDTree class accesses this member
	static const int DIM = 2;

	// the constructors
	LocalPoint() {}
	LocalPoint(double x, double y)
	{ 
		(*this)[0] = x;
		(*this)[1] = y;
	}

	// conversion to OpenCV Point2d
	operator cv::Point2d() const { return cv::Point2d((*this)[0], (*this)[1]); }
};

bool ZigZagCpp(ram_path_planning::Cpp::Request& req,
			   ram_path_planning::Cpp::Response& res){
  	if (req.height_between_layers <= 0)
  	{
    	ROS_ERROR_STREAM("Height between layers cannot be <= 0");
    	return false;
 	}

  	if (req.deposited_material_width <= 0)
  	{
    	ROS_ERROR_STREAM("Deposited material width cannot be <= 0");
    	return false;
  	}

  	if (req.contours_filtering_tolerance < 0)
  	{
    	ROS_ERROR_STREAM("Contours filtering tolerance cannot be < 0");
    	return false;
  	}

  	if(req.map.data.size() != req.map.info.width*req.map.info.height){
		ROS_ERROR_STREAM("Bad map data...");
		return false;
	}

	/*if(req.transform.size() != 3*3){
		ROS_ERROR_STREAM("Bad transform matrix...");
		return false;
	}*/

	if(req.internal_contour_threshold <= 0){
		ROS_ERROR_STREAM("internal contour threshold cannot be <=0");
	}
	
	for(int i = 0;i < req.map.data.size();i++){
		if(req.map.data[i] == -1)
			req.map.data[i] = 100;
	}

	cv::Mat map(req.map.info.height, req.map.info.width, CV_8UC1, req.map.data.data());

	cv::Mat bin;
	cv::threshold(map,bin,req.occupancy_threshold,255,cv::THRESH_BINARY_INV);
	
    double delta_point = 1;
	std::vector<cv::Point2i> vertices_point;
	vertices_point = makeOIP(bin,delta_point);
    
    std::vector<cv::Point> contour_ext;
    cv::approxPolyDP(vertices_point,contour_ext,3.0,true);
    
    //B+Tree
    NODE* head = NULL;
    head = new NODE(contour_ext.size(),0,0);

    ROS_INFO_STREAM("head->polygon:" << head->polygon);
    ROS_INFO_STREAM("head->n_children:" << head->n_children);
    ROS_INFO_STREAM("head->n_point:" << head->n_point);
    ROS_INFO_STREAM("head->level:" << head->level);
    ROS_INFO_STREAM("head->children:" << head->children);

    PolygonCgal polyCgalPoint;
    std::vector<LocalPoint> extPoint;
	for(int j = 0;j < contour_ext.size();j++){
		double point_x = contour_ext[j].x*req.map_resolution+req.map_origin_x;
		double point_y = contour_ext[j].y*req.map_resolution+req.map_origin_y;
				
		//int point_x = vertices_point[j].x;
		//int point_y = vertices_point[j].y;
		
        geometry_msgs::Pose pose_;
		pose_.position.x = point_x;
		pose_.position.y = point_y;

		//res.pose.push_back(pose_);	
        polyCgalPoint.push_back(PointCgal(contour_ext[j].x,contour_ext[j].y));
    //std::cout << __FILE__ << __LINE__ << std::endl;
        head->polygon[j].x = point_x;
        head->polygon[j].y = point_y;
    //std::cout << __FILE__ << __LINE__ << std::endl;
        extPoint.push_back(LocalPoint(point_x,point_y));
	}
    
    //std::cout << __FILE__ << __LINE__ << std::endl;
    /*if(polyCgalPoint.is_clockwise_oriented())
        polyCgalPoint.reverse_orientation();*/

    //std::cout << __FILE__ << __LINE__ << std::endl;
    for(int i = 0;i < head->n_children;i++){
       ROS_INFO_STREAM("head->polygon:(" << head->polygon[i].x << "," << head->polygon[i].y << ")");
    }
    
    /*PolygonWithHolesCgal polyWithHoles(poly_cgal);
    polyWithHoles.outer_boundary() = polyCgalPoint;
    
    SsPtr iss = CGAL::create_interior_straight_skeleton_2(polyWithHoles);
    ROS_INFO_STREAM("iss.size_of_vertices():" << iss->size_of_vertices());
    ROS_INFO_STREAM("iss.size_of_halfedges():" << iss->size_of_halfedges());
    ROS_INFO_STREAM("iss.size_of_faces():" << iss->size_of_faces());*/
    
    //std::cout << __FILE__ << __LINE__ << std::endl;
    double lOffset = 0.2;
    //PolygonPtrVector offset_polygonsncident vertex of h.  = CGAL::create_offset_polygons_2<PolygonCgal>(lOffset,*iss);
    PolygonPtrVector offset_polygons;
    //offset_polygons = CGAL::create_interior_skeleton_and_offset_polygons_2(lOffset,poly_cgal);
    //ROS_INFO_STREAM("PoygonPtrVectorSize:" << offset_polygons.size());
    bool flag = false;
    int count_level = -1;
    std::vector<std::vector<NODE*> > tempNode;
    //std::cout << __FILE__ << __LINE__ << std::endl;
    std::vector<NODE*> tempNodeInit;
    //std::cout << __FILE__ << __LINE__ << std::endl;
    tempNodeInit.push_back(head);
    //std::cout << __FILE__ << __LINE__ << std::endl;
    tempNode.push_back(tempNodeInit);
    //std::cout << __FILE__ << __LINE__ << std::endl;
    while(!flag){
        flag = true;

    //std::cout << __FILE__ << __LINE__ << std::endl;
        if(tempNode.size() > 0){
            count_level++;
            std::vector<NODE*> tempNode_;
            for(NODE* node : tempNode[count_level]){
                  
    //std::cout << __FILE__ << __LINE__ << std::endl;
                PolygonCgal poly_cgal;
                PolygonCgalToPtr(node->polygon,node->n_point,poly_cgal);
                ROS_INFO_STREAM("poly_cgal_size:" << poly_cgal.size());
                
                if(poly_cgal.is_clockwise_oriented())
                    poly_cgal.reverse_orientation();
                
                for(int i = 0;i < poly_cgal.size();i++){
                    PointCgal p = poly_cgal.vertex(i);
                    ROS_INFO_STREAM("p:(" << p.x() << "," << p.y() << ")");
                }

                offset_polygons = CGAL::create_interior_skeleton_and_offset_polygons_2(lOffset,poly_cgal);
    //std::cout << __FILE__ << __LINE__ << std::endl;
                //if(offset_polygons.size() > 0)
                    //count_level++;
                ROS_INFO_STREAM("offset_polygons_size:" << offset_polygons.size());
                if(offset_polygons.size() == 0){
                    flag &= true;
                    continue;
                }
                else
                    flag &= false;

                node->n_children = offset_polygons.size();
                node->children = new NODE* [offset_polygons.size()];
                int index = 0;

                //std::vector<NODE*>(tempNode).swap(tempNode);
    //std::cout << __FILE__ << __LINE__ << std::endl;
                //std::vector<NODE*> tempNode_;
                for(PolygonPtrVector::const_iterator pi = offset_polygons.begin() ; pi != offset_polygons.end() ; ++ pi){
               
                    PolygonPtr pi_ = *pi;
                    PolygonCgal pi_cgal = *pi_;
                    NODE* node_cell = new NODE(pi_cgal.size(),0,count_level+1);
                    
                    ROS_INFO_STREAM("node_cell->polygon:" << node_cell->polygon);
                    ROS_INFO_STREAM("node_cell->n_children:" << node_cell->n_children);
                    ROS_INFO_STREAM("node_cell->n_point:" << node_cell->n_point);
                    ROS_INFO_STREAM("node_cell->level:" << node_cell->level);
                    ROS_INFO_STREAM("node_cell->children:" << node_cell->children);

                    for(int i = 0;i < pi_cgal.size();i++){
                        PointCgal p = pi_cgal.vertex(i);
                        //ROS_INFO_STREAM("p:(" << p.x() << "," << p.y() << ")");
                        node_cell->polygon[i].x = p.x();
                        node_cell->polygon[i].y = p.y();
                    }

                    //node_cell->polygon = pi_cgal;
                    //node_cell->level = count_level;
                    node->children[index++] = node_cell;
                    tempNode_.push_back(node_cell);
                }
    //std::cout << __FILE__ << __LINE__ << std::endl;
                
            }
            
            tempNode.push_back(tempNode_);
    //std::cout << __FILE__ << __LINE__ << std::endl;
        }
        
        
        /*offset_polygons = CGAL::create_interior_skeleton_and_offset_polygons_2(lOffset,poly_cgal);
        ROS_INFO_STREAM("PoygonPtrVectorSize:" << offset_polygons.size());
        
        for(PolygonPtrVector::const_iterator pi = offset_polygons.begin() ; pi != offset_polygons.end() ; ++ pi){
             
            PolygonPtr pi_ = *pi;
            PolygonCgal pi_cgal = *pi_;
            
            geometry_msgs::Polygon partial_polygon;

            ROS_INFO_STREAM("pi_cgal.size:" << pi_cgal.size());
            
            std::vector<cv::Point2f> pi_cgal_te;
            for(int i = 0;i < pi_cgal.size();i++){
                 geometry_msgs::Point32 point_32;
                 PointCgal p = pi_cgal.vertex(i);

                 point_32.x = p.x();
                 point_32.y = p.y();
                 partial_polygon.points.push_back(point_32);
                 poly_temp.push_back(Point_2(p.x(),p.y()));
                 pi_cgal_te.push_back(cv::Point2f(p.x(),p.y()));
            }
            
            pi_cgal_ex.push_back(pi_cgal_te);
            res.polygon.push_back(partial_polygon);
        }*/
        
    }
    
    //BFS
    ROS_INFO_STREAM("tempNode_size:" << tempNode.size());
    for(int i = 0;i < tempNode.size();i++){
        //ROS_INFO_STREAM("tempNode_size:" << tempNode.size());
        //geometry_msgs::Polygon partial_polygon;
        ROS_INFO_STREAM("tempNode_[" << i << "]:" << tempNode[i].size());
        for(int j = 0;j < tempNode[i].size();j++){
            geometry_msgs::Polygon partial_polygon;
            ROS_INFO_STREAM("tempNode_level:" << tempNode[i][j]->level);
            for(int k = 0;k < tempNode[i][j]->n_point;k++){
                geometry_msgs::Point32 point_32;
                point_32.x = tempNode[i][j]->polygon[k].x;
                point_32.y = tempNode[i][j]->polygon[k].y;

                partial_polygon.points.push_back(point_32);
            }
            res.polygon.push_back(partial_polygon);
        }
    }
    
    //delete
    BfsTree(head);
    
   
    //straight skeleton
    if(polyCgalPoint.is_clockwise_oriented())
        polyCgalPoint.reverse_orientation();

    PolygonWithHolesCgal polyWithHoles(polyCgalPoint);
    polyWithHoles.outer_boundary() = polyCgalPoint;
    
    SsPtr iss = CGAL::create_interior_straight_skeleton_2(polyWithHoles);
    ROS_INFO_STREAM("iss.size_of_vertices():" << iss->size_of_vertices());
    ROS_INFO_STREAM("iss.size_of_halfedges():" << iss->size_of_halfedges());
    ROS_INFO_STREAM("iss.size_of_faces():" << iss->size_of_faces());
    
    std::map<SSEdge,int> SSEdgeTop;
    int key_ = 0;
    if(iss){
        for(auto face = iss->faces_begin();face != iss->faces_end();face++){
            Ss::Halfedge_const_handle begin = face->halfedge();
            Ss::Halfedge_const_handle edge = begin;

            do{
                
                Ss::Halfedge_const_handle opposite_edge = edge->opposite();
                const Vertex_const_handle& v = edge->vertex();
                const Vertex_const_handle& v_op = opposite_edge->vertex();
                
                cv::Point from;
                cv::Point to;
                bool isSplitNodeFrom = false;
                bool isSplitNodeTo = false;
                
                from.x = v->point().x();
                from.y = v->point().y();
                to.x = v_op->point().x();
                to.y = v_op->point().y();

                //SP sp_(v->point().x(),v->point().y(),v->time());
                //SP sp_op(v_op->point().x(),v_op->point().y(),v_op->time());
                
                SSEdge ss_edge(from,to,isSplitNodeFrom,isSplitNodeTo);
                ROS_INFO_STREAM("#####################################################");
                ROS_INFO_STREAM("map.count:" << SSEdgeTop.count(ss_edge));
                
                ROS_INFO_STREAM("v_is_skeleton:" << v->is_skeleton());
                ROS_INFO_STREAM("from:(" << from.x << "," << from.y << ")");
                
                ROS_INFO_STREAM("v_op_is_contour:" << v_op->is_contour());
                ROS_INFO_STREAM("to:(" << to.x << "," << to.y << ")");

                if(v->is_skeleton() && v_op->is_contour())
                {
                    if(SSEdgeTop.count(ss_edge) == 0){
                        double t = v->time();
                        if(t < 9.0){
                            ss_edge.isSplitNodeFrom = true;
                            ss_edge.time = t;

                            ROS_INFO_STREAM("v_t:" << t);
                            geometry_msgs::Pose p_t;
                            p_t.position.x = v->point().x()*req.map_resolution+req.map_origin_x;
                            p_t.position.y = v->point().y()*req.map_resolution+req.map_origin_y;
                            p_t.position.z = 0.f;

                            ROS_INFO_STREAM("p_t:(" << p_t.position.x << "," << p_t.position.y << ")");
                            res.point_skeleton.push_back(p_t);

                            SSEdgeTop.insert(std::make_pair(ss_edge,key_++));
                        }
                    }
                    /*if(skeletonPointMap.count(sp_) == 0){
                        ROS_INFO_STREAM("collapse time:" << sp_.t);

                        geometry_msgs::Pose p_t;
                        p_t.position.x = v->point().x()*req.map_resolution+req.map_origin_x;;
                        p_t.position.y = v->point().y()*req.map_resolution+req.map_origin_y;;
                        p_t.position.z = 0.f;

                        ROS_INFO_STREAM("p_t:(" << p_t.position.x << "," << p_t.position.y << ")");
                        res.point_skeleton.push_back(p_t);

                        skeletonPointMap.insert(std::make_pair(sp_,key_++));
                    }*/
                }
                /*else if(v->is_contour() && v_op->is_skeleton()){
                   double t = v_op->time();
                   if(t < 10.0){
                        ss_edge.isSplitNodeTo = true;
                        ss_edge.time = t;

                        ROS_INFO_STREAM("v_op_t:" << t);
                        geometry_msgs::Pose p_t;
                        p_t.position.x = v_op->point().x()*req.map_resolution+req.map_origin_x;;
                        p_t.position.y = v_op->point().y()*req.map_resolution+req.map_origin_y;;
                        p_t.position.z = 0.f;

                        ROS_INFO_STREAM("p_t:(" << p_t.position.x << "," << p_t.position.y << ")");
                        res.point_skeleton.push_back(p_t);

                        SSEdgeTop.insert(std::make_pair(ss_edge,key_++));
                   }
                }*/
                
                edge = edge->prev();

            }while(edge != begin);
        }
    }
    
    ROS_INFO_STREAM("key_:" << key_);
    
    ROS_INFO_STREAM("$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$");    
    
    //init KDtree
    KDTree<LocalPoint> kdtree(extPoint);
    
    //store spilt edge
    std::map<SPEdge,int>::iterator iter_sp_edge;
    std::map<SPEdge,int,edgeCmp> SPEdgeTop;

    std::map<SSEdge,int>::iterator iter_edge;
    double sub_area_limit = 1.0;
    double split_edge_limit = 0.5;
    int split_key = 0;

    for(iter_edge = SSEdgeTop.begin();iter_edge != SSEdgeTop.end();iter_edge++){
        LocalPoint query(iter_edge->first.from.x*req.map_resolution+req.map_origin_x,
                         iter_edge->first.from.y*req.map_resolution+req.map_origin_y);
        ROS_INFO_STREAM("*************************************************************************");
        ROS_INFO_STREAM("from:(" << iter_edge->first.from.x*req.map_resolution+req.map_origin_x << ","
                                 << iter_edge->first.from.y*req.map_resolution+req.map_origin_y << ")");
        
        const std::vector<int> knnIndices = kdtree.knnSearch(query, 2);
        int start_id = knnIndices[0] < knnIndices[1]?knnIndices[0]:knnIndices[1];
        int end_id = knnIndices[0] > knnIndices[1]?knnIndices[0]:knnIndices[1];
        
        SPEdge sp_edge(start_id,end_id);

        /*ROS_INFO_STREAM("ID:(" << begin_id << "," << end_id << ")");
        for(int i : knnIndices){
            ROS_INFO_STREAM("kdtree:(" << cv::Point2d(extPoint[i]).x << "," << cv::Point2d(extPoint[i]).y << ")");
            geometry_msgs::Pose p_t;
            p_t.position.x = cv::Point2d(extPoint[i]).x;
            p_t.position.y = cv::Point2d(extPoint[i]).y;
            p_t.position.z = 0.f;
            
            res.point_nearest.push_back(p_t);
        }*/

        ROS_INFO_STREAM("ID:(" << start_id<< "," << end_id << ")");
        ROS_INFO_STREAM("SPEdgeTop.count(sp_edge):" << SPEdgeTop.count(sp_edge));
        if(SPEdgeTop.count(sp_edge) == 0){
            std::vector<cv::Point2f> sub_contour;
            for(int i = sp_edge.start_id;i < sp_edge.end_id;i++){
                sub_contour.push_back(cv::Point2f(cv::Point2d(extPoint[i]).x,cv::Point2d(extPoint[i]).y));

                ROS_INFO_STREAM("sub_contour:(" << cv::Point2d(extPoint[i]).x << "," 
                                               << cv::Point2d(extPoint[i]).y << ")");
            }

            ROS_INFO_STREAM("sub_contour_size:" << sub_contour.size());

            if(sub_contour.size() < 3)
                continue;

            double sub_area = cv::contourArea(sub_contour,false);
            ROS_INFO_STREAM("sub_area:" << sub_area);

            if(sub_area > sub_area_limit){
                double delta_x = cv::Point2d(extPoint[sp_edge.start_id]).x-cv::Point2d(extPoint[sp_edge.end_id]).x;
                double delta_y = cv::Point2d(extPoint[sp_edge.start_id]).y-cv::Point2d(extPoint[sp_edge.end_id]).y;
                ROS_INFO_STREAM("split_edge_limit:" << std::sqrt(delta_x*delta_x + delta_y*delta_y));

                if(std::sqrt(delta_x*delta_x + delta_y*delta_y) > split_edge_limit){
                    SPEdgeTop.insert(std::make_pair(sp_edge,split_key++));      

                    ROS_INFO_STREAM("kdtree_start:(" << cv::Point2d(extPoint[sp_edge.start_id]).x << "," 
                                               << cv::Point2d(extPoint[sp_edge.start_id]).y << ")");
                    geometry_msgs::Pose p_t;
                    p_t.position.x = cv::Point2d(extPoint[sp_edge.start_id]).x;
                    p_t.position.y = cv::Point2d(extPoint[sp_edge.start_id]).y;
                    p_t.position.z = 0.f;
                    
                    res.point_nearest.push_back(p_t);

                    ROS_INFO_STREAM("kdtree_end:(" << cv::Point2d(extPoint[sp_edge.end_id]).x << "," 
                                               << cv::Point2d(extPoint[sp_edge.end_id]).y << ")");
                    geometry_msgs::Pose p_t_end;
                    p_t_end.position.x = cv::Point2d(extPoint[sp_edge.end_id]).x;
                    p_t_end.position.y = cv::Point2d(extPoint[sp_edge.end_id]).y;
                    p_t_end.position.z = 0.f;
                    
                    res.point_nearest.push_back(p_t_end);
                }
            }
        }

        /*ROS_INFO_STREAM("to:(" << iter_edge->first.to.x*req.map_resolution+req.map_origin_x << ","
                               << iter_edge->first.to.y*req.map_resolution+req.map_origin_y << ")");
        ROS_INFO_STREAM("isSplitNodeFrom:" << iter_edge->first.isSplitNodeFrom << ",isSplitNodeTo:" << iter_edge->first.isSplitNodeTo);
        ROS_INFO_STREAM("time:" << iter_edge->first.time);
        ROS_INFO_STREAM("$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$");*/


    }
    
	/*std::vector<std::vector<cv::Point> > contours;
	std::vector<cv::Vec4i> hierarchy;

	cv::findContours(bin,contours,hierarchy,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE,cv::Point());
    ROS_INFO_STREAM("contours_size:" << contours.size());
	
    
    int external_contour_id;
	int max_external_contour = 0;
	//ext
	for(int i = 0;i < contours.size();i++){
        ROS_INFO_STREAM("contour_obj:" << contours[i].size());
		if(contours[i].size() > max_external_contour){
			max_external_contour = contours[i].size();
			external_contour_id = i;
		}
	}
    
    std::vector<cv::Point> contour_ext_dp;
    cv::approxPolyDP(contours[external_contour_id],contour_ext_dp,12.0,true);

    PolygonCgal poly_cgal;
	for(int j = 0;j < contour_ext_dp.size();j++){
		double point_x = contour_ext_dp[j].x*req.map_resolution+req.map_origin_x;
		double point_y = contour_ext_dp[j].y*req.map_resolution+req.map_origin_y;
				
		geometry_msgs::Pose pose_;
		pose_.position.x = point_x;
		pose_.position.y = point_y;

		res.pose.push_back(pose_);	
        poly_cgal.push_back(Point_2(point_x,point_y));
	}
    
    PolygonWithHolesCgal polyHoles(poly_cgal);
    polyHoles.outer_boundary() = poly_cgal;*/
#if 0
	ROS_INFO("valid_external_contour_size:%d",contours[external_contour_id].size());
	//find subcontour
	for(int i = 0;i < contours.size();i++){
        if(i != external_contour_id)
			if(contours[i].size() > req.internal_contour_threshold)
            {
				valid_internal_contours.push_back(contours[i]);
				ROS_INFO("valid_internal_contour_size:%d",contours[i].size());
			}
	}

	ROS_INFO("valid_internal_contour_number:%d",valid_internal_contours.size());
	//add internal contour data
	for(int i = 0;i < valid_internal_contours.size();i++){
        PolygonCgal holes;
        
        cv::Rect rect = cv::boundingRect(valid_internal_contours[i]);
        
        double point_top_left_x = rect.x*req.map_resolution+req.map_origin_x;
        double point_top_left_y = rect.y*req.map_resolution+req.map_origin_y;
        double point_top_right_x = (rect.x+rect.width)*req.map_resolution+req.map_origin_x;
        double point_top_right_y = rect.y*req.map_resolution+req.map_origin_y;
        double point_bot_left_x = rect.x*req.map_resolution+req.map_origin_x;
        double point_bot_left_y = (rect.y+rect.height)*req.map_resolution+req.map_origin_y;
        double point_bot_right_x = (rect.x+rect.width)*req.map_resolution+req.map_origin_x;
        double point_bot_right_y = (rect.y+rect.height)*req.map_resolution+req.map_origin_y;

        holes.push_back(PointCgal(point_top_left_x,point_top_left_y));
        holes.push_back(PointCgal(point_top_right_x,point_top_right_y));
        holes.push_back(PointCgal(point_bot_right_x,point_bot_right_y));
        holes.push_back(PointCgal(point_bot_left_x,point_bot_left_y));

        /*cv::RotatedRect rRect = cv::minAreaRect(valid_internal_contours[i]);
		cv::Point2f vertices[4];
		rRect.points(vertices);

        for(int i = 0;i < 4;i++){
			double point_x = vertices[i].x*req.map_resolution+req.map_origin_x;
			double point_y = vertices[i].y*req.map_resolution+req.map_origin_y;

            holes.push_back(PointCgal(point_x,point_y));
		}*/

		/*std::vector<cv::Point> convex_contour_poly_p;
		cv::convexHull(valid_internal_contours[i],convex_contour_poly_p,false,true);

		for(int i = 0;i < convex_contour_poly_p.size();i++){
			double point_x = convex_contour_poly_p[i].x*req.map_resolution+req.map_origin_x;
			double point_y = convex_contour_poly_p[i].y*req.map_resolution+req.map_origin_y;
            
            holes.push_back(PointCgal(point_x,point_y));
		}*/

        ROS_INFO("hole_size:%d",holes.size());
        polyHoles.add_hole(holes);
	}
#endif

    //PolygonListCgal partition_polys_;
    //PolygonListCgal partition_polys;
    
    /*std::vector<PointVector> subPolygons;
    std::vector<PointVector> final_path;
	nav_msgs::Path plan_path;
    std_msgs::Float64 footprintLength, footprintWidth, horizontalOverwrap, verticalOverwrap;
    footprintLength.data = step_length;
    footprintWidth.data = step_length; 
    horizontalOverwrap.data = step_overlap;
    verticalOverwrap.data = step_overlap;*/

	// Generate trajectory
  	/*if (true)
  	{
        std::cout << __FILE__ << __LINE__ << std::endl;
        CGAL::Polygon_vertical_decomposition_2<Kernel,ContainerCgal> obj;
        std::cout << __FILE__ << __LINE__ << std::endl;
        obj(polyHoles,std::back_inserter(partition_polys));
        std::cout << __FILE__ << __LINE__ << std::endl;

        //partition_polys.sort(compare_polygon);

        std::list<PolygonCgal>::iterator iter;
        for(iter = partition_polys.begin();iter != partition_polys.end();iter++){
            geometry_msgs::Polygon partial_polygon;
            PointVector polygon_bcd,candidatePath;

            PolygonCgal poly_cell = *iter;
            for(int j = 0;j < poly_cell.size();j++){
                geometry_msgs::Point32 point_32;
                geometry_msgs::Point point_divide_bcd;

                PointCgal p = poly_cell.vertex(j);

                point_32.x = p.x();
                point_32.y = p.y();
                point_divide_bcd.x = p.x();
                point_divide_bcd.y = p.y();
                
                polygon_bcd.push_back(point_divide_bcd);

                partial_polygon.points.push_back(point_32);
             }
             res.polygon.push_back(partial_polygon);
                
        }
  	}*/

	return true;
}

}


int main(int argc, char **argv) {
  ros::init(argc, argv, "cpp_node");
  ros::NodeHandle private_nh("~");

  ros::ServiceServer zigzag_cpp_srv = private_nh.advertiseService(
      "/sweeper/zigzag_cpp_srv",
      Cpp::ZigZagCpp);

  ROS_INFO("Ready to make zigzag cpp...");
  ros::spin();

  return 0;
}

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
#include <CGAL/Boolean_set_operations_2.h>
#include <CGAL/Polygon_set_2.h>
#include <tuple>
#include <limits.h>

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
typedef std::list<PolygonWithHolesCgal> PolygonWithHolesListCgal;
typedef std::vector<PointCgal> ContainerCgal;
typedef CGAL::Straight_skeleton_2<K>  Ss;
typedef boost::shared_ptr<Ss> SsPtr;
typedef CGAL::Straight_skeleton_2<K>::Vertex_const_handle     Vertex_const_handle ;
typedef CGAL::Straight_skeleton_2<K>::Halfedge_const_handle   Halfedge_const_handle ;
typedef boost::shared_ptr<PolygonCgal> PolygonPtr;
typedef std::vector<PolygonPtr> PolygonPtrVector;
typedef CGAL::Polygon_set_2<Kernel> Polygon_set_2_cgal;
typedef Kernel::Segment_2 Segment_Cgal;
typedef Kernel::Line_2 Line_Cgal;
typedef Kernel::Intersect_2 Intersect_Cgal;

#define INF 1000000000

/*int n_cycle = 0;
bool* marked;
bool* onStack;
std::tuple<int,int,double>* edgeTo;
std::stack<std::tuple<int,int,double>>* cycle;*/

std::stack<int> s_node;
bool* visited;
std::vector<std::stack<int> > DPaths;
std::map<int, std::vector<std::tuple<int, int,double>>> EWD;

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
     
        //ROS_INFO_STREAM("ptr:" << p);
        for(int i = 0;i < p->n_children;i++){
            q.push(p->children[i]);
            //ROS_INFO_STREAM("ptr_:" << p->children[i]);
            s.push(p->children[i]);
        }
        //ROS_INFO_STREAM("*************************************************");
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

typedef struct SP_Union{
    cv::Point2f p;
    cv::Point2f c1;
    int c1_index;
    cv::Point2f c2;
    int c2_index;
    std::vector<int> sub_contour;

    SP_Union(cv::Point2f& p_,cv::Point2f& c1_,int c1_index_,cv::Point2f& c2_,int c2_index_,std::vector<int>& sub_contour_){
        p.x = p_.x;
        p.y = p_.y;

        c1.x = c1_.x;
        c1.y = c1_.y;
        
        c2.x = c2_.x;
        c2.y = c2_.y;

        c1_index = c1_index_;
        c2_index = c2_index_;

        for(int i = 0;i < sub_contour_.size();i++)
            sub_contour.push_back(sub_contour_[i]);
    }
    
    bool operator < (const SP_Union& u)const
    {
        if(sub_contour.size() < u.sub_contour.size())
            return true;
        else if(sub_contour.size() > u.sub_contour.size())
            return false;
        else{
            return (p.x < u.p.x?true:false);
        }
    }

}SPUnion;

typedef struct SP_Point{
    int index;

    SP_Point(int index_){
        index = index_;
    }

    bool operator < (const SP_Point& p_)const
    {
        return (index < p_.index?true:false);
    }
}SPPoint;

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

/*bool isOverlapPolygon(std::vector<int>& a,std::vector<int>& b){
    for(int i = 0;i < a.size();i++){
        for(int j = 0;j < b.size();j++){
            if(a[i] == b[j])
                return true;
        }
    }

    return false;
}*/

typedef struct overlap_index{
    int p;
    std::vector<int> p_index;
    overlap_index(int p_,std::vector<int>& p_index_)
    {
        p = p_;

        for(int i = 0;i < p_index_.size();i++){
            p_index.push_back(p_index_[i]);
        }
    }

}OIndex;

void print_polygon (const PolygonCgal& P){
    PolygonCgal::Vertex_const_iterator vit;
    std::cout << "[ " << P.size() << " vertices:";
    for (vit = P.vertices_begin(); vit != P.vertices_end(); ++vit)
        std::cout << " (" << *vit << ')';
    std::cout << " ]" << std::endl;
}

typedef struct splitPoint{
    cv::Point p;

    splitPoint(cv::Point& p_){
        p.x = p_.x;
        p.y = p_.y;
    }

    bool operator < (const splitPoint& p_)const
    {
        if(p.x < p_.p.x)
            return true;
        else if(p.x == p_.p.x){
            return (p.y < p_.p.y?true:false);
        }
        else
            return false;
    }
}SplitPoint;

void splitPolygon(std::vector<PolygonCgal>& poly_union,int a,int b,std::vector<PolygonCgal>& res,double ox,double oy,double r){
    //res.push_back(poly_union[a]);
    
    //print_polygon(poly_union[a]);
    PolygonCgal a_p;
    std::map<SplitPoint,int> map_a;
    int split_point_key = 0;
    for(int i = 1;i < poly_union[a].size()-1;i++){
        PointCgal p = poly_union[a].vertex(i);
        int p_x = (p.x()-ox)/r;
        int p_y = (p.y()-oy)/r;

        a_p.push_back(PointCgal(p_x,p_y));
        cv::Point cv_p(p_x,p_y);

        SplitPoint split_point(cv_p);
        map_a.insert(std::make_pair(split_point,split_point_key++));
    }

    //print_polygon(a_p);

    
    PolygonCgal b_p;
    for(int i = 0;i < poly_union[b].size();i++){
        PointCgal p = poly_union[b].vertex(i);
        int p_x = (p.x()-ox)/r;
        int p_y = (p.y()-oy)/r;

        b_p.push_back(PointCgal(p_x,p_y));
    }

    //print_polygon(poly_union[b]);
    //print_polygon(b_p);
    //PolygonWithHolesListCgal diff;
    //PolygonWithHolesListCgal::const_iterator it;
    //CGAL::symmetric_difference(poly_union[b],poly_union[a],std::back_inserter(diff));
    //CGAL::difference(poly_union[b],poly_union[a],std::back_inserter(diff));
   
    //ROS_INFO_STREAM("diff_size:" << diff.size());
    
    //for(it = diff.begin();it != diff.end();it++){
        //ROS_INFO_STREAM("is_unbounded:" << it->is_unbounded());
        //if(!it->is_unbounded()){
            //print_polygon(it->outer_boundary());
            //PolygonCgal temp;

            //for(int j = 0;j < it->outer_boundary().size();j++){
                //PointCgal p = it->outer_boundary().vertex(j);
                //temp.push_back(PointCgal(p.x(),p.y()));
            //}
            
            /*if(temp.is_counterclockwise_oriented())
                temp.reverse_orientation();*/

            //res.push_back(temp);
            
            /*std::cout << " " << it->number_of_holes() << " holes:" << std::endl;
            int k = 1;

            PolygonWithHolesCgal::Hole_const_iterator hit;
            for(hit = it->holes_begin(); hit != it->holes_end(); ++hit, ++k){
                std::cout << " Hole #" << k << " = ";
                print_polygon (*hit);

            }*/
        //}
    //}
    PolygonCgal temp;
    for(int i = 0;i < poly_union[b].size();i++){
        PointCgal p = poly_union[b].vertex(i);
        int p_x = (p.x()-ox)/r;
        int p_y = (p.y()-oy)/r;

        cv::Point cv_p(p_x,p_y);

        SplitPoint split_point(cv_p);

        if(map_a.count(split_point) == 0)
            temp.push_back(PointCgal(p.x(),p.y()));
    }
    
    res.push_back(temp);
    //ROS_INFO_STREAM("temp_size:" << temp.size());
    //print_polygon(temp);
}

bool IsRoot(int index_,std::vector<SPEdge>& vec_,SPEdge& e_){
    for(int i = 0;i < vec_.size();i++){
        if((vec_[i].start_id == index_) || (vec_[i].end_id == index_)){
            e_.start_id = vec_[i].start_id;
            e_.end_id = vec_[i].end_id;
            return true;
         }
    }

    return false;
}

/*void dfsCycle(int v) 
{
    onStack[v] = true;
    marked[v] = true;

    for(std::vector<std::tuple<int, int, double>>::iterator ii = EWD[v].begin(); ii != EWD[v].end(); ii++)
    {
        int w = std::get<1>(*ii);

        if(!marked[w]) {
            if(v == 0)
                edgeTo[0] = std::make_tuple(EWD.size()-1,0,1);

            edgeTo[w] = *ii;
                
            std::cout << "w_d:" << w << std::endl;
            dfsCycle(w);
        }
        else if(onStack[w]){
            std::tuple<int, int, double> f = *ii;
            std::cout << "w_c:" << w << std::endl;
            std::cout << __FILE__ << __LINE__ << std::endl;
            while(std::get<0>(f) != w) {
                //std::cout << "w_c:" << w << std::endl;
                //std::cout << "std::get<0>(f):" << std::get<0>(f) << std::endl;
                cycle[n_cycle].push(f);
                f = edgeTo[std::get<0>(f)];
            }

            std::cout << __FILE__ << __LINE__ << std::endl;
            cycle[n_cycle].push(f);
            n_cycle++;
            std::cout << "n_cycle: " << n_cycle << std::endl;
            std::cout << __FILE__ << __LINE__ << std::endl;
            return ;
        }
    }

    onStack[v] = false;
}*/

/*void Dijkstra(int s,
              int maxv,
              int* d,
              bool* vis,
              std::map<int, std::vector<std::tuple<int, int,double>>>& EWD_,
              int* pre){
    std::fill(d,d+maxv,INF);
    for(int i = 0;i < maxv;i++)
        std::cout << "d:" << d[i] << std::endl;

    std::fill(vis,vis+maxv,0);
    for(int i = 0;i < maxv;i++)
        std::cout << "vis:" << vis[i] << std::endl;
    
    for(int i = 0;i < maxv;i++){
        pre[i] = i;
    }

    d[s] = 0;
    
    for(int i = 0;i < maxv;i++){
        int u = -1;
        int MIN = INF;

        for(int j = 0;j < n;j++){
            if(vis[j] == false && d[j] < MIN){
                u = j;
                MIN = d[j];
            }
        }

        if(u == -1){
            ROS_INFO_STREAM("Dijkstra fail!!!");
            return;
        }

        vis[u] = true;

        for(int j = 0;j < EWD_[u].size();j++){
            int v = std::get<1>(EWD_[u][j]);
            double d_ = std::get<2>(EWD_[u][j]);
            if(vis[v] == false && d[u]+d_ < d[v]){
                d[v] = d[u]+d_;
                pre[v] = u;
            }
        }

    }

}*/

void DfsFindPaths(int start,int end){   
    visited[start] = true;
    s_node.push(start);

    for(int j = 0;j < EWD[start].size();j++){
        if(std::get<0>(EWD[start][j]) == end){
            //s_node.push(end);

            std::stack<int> path;
            while(!s_node.empty()){
                path.push(s_node.top());
                s_node.pop();
            }
            
            DPaths.push_back(path);

            while(!path.empty()){
                s_node.push(path.top());
                path.pop();
            }
           
            s_node.pop();
            visited[end] = false;
            
            break;
        }

        if(!visited[std::get<1>(EWD[start][j])])
            DfsFindPaths(std::get<1>(EWD[start][j]),end);

        if(j == (EWD[start].size()-1)){
            s_node.pop();
            visited[start] = false;
        }
    }
}

int FindSplitPath(std::vector<std::stack<int> >& vec){
    int MaxNumber = INT_MAX;
    int index;
    for(int i = 0;i < vec.size();i++){
        if(vec[i].size() == 2)
            continue;
        else{
            if(vec[i].size() < MaxNumber){
                index = i;
                MaxNumber = vec[i].size();
            }
        }
    }

    return index;
}

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
    cv::approxPolyDP(vertices_point,contour_ext,2.5,true);
    //ROS_INFO_STREAM("contour_ext:" << contour_ext.size());

    //B+Tree
    NODE* head = NULL;
    head = new NODE(contour_ext.size(),0,0);

    /*ROS_INFO_STREAM("head->polygon:" << head->polygon);
    ROS_INFO_STREAM("head->n_children:" << head->n_children);
    ROS_INFO_STREAM("head->n_point:" << head->n_point);
    ROS_INFO_STREAM("head->level:" << head->level);
    ROS_INFO_STREAM("head->children:" << head->children);*/

    //PolygonCgal polyCgalExt;
    std::vector<SplitPoint> polyCgalExt;

    PolygonCgal polyCgalPoint;
    std::vector<LocalPoint> extPoint;
    PolygonCgal polyTest;

	for(int j = 0;j < contour_ext.size();j++){
		double point_x = contour_ext[j].x*req.map_resolution+req.map_origin_x;
		double point_y = contour_ext[j].y*req.map_resolution+req.map_origin_y;
				
		int p_x = contour_ext[j].x;
		int p_y = contour_ext[j].y;
		cv::Point cv_p(p_x,p_y);

        SplitPoint split_point(cv_p);
        polyCgalExt.push_back(split_point);

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
        polyTest.push_back(PointCgal(p_x,p_y));
	}
    
    //std::cout << __FILE__ << __LINE__ << std::endl;
    //if(polyCgalExt.is_clockwise_oriented())
        //polyCgalExt.reverse_orientation();

    //std::cout << __FILE__ << __LINE__ << std::endl;
    /*for(int i = 0;i < head->n_children;i++){
       ROS_INFO_STREAM("head->polygon:(" << head->polygon[i].x << "," << head->polygon[i].y << ")");
    }*/
    
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
                //ROS_INFO_STREAM("poly_cgal_size:" << poly_cgal.size());
                
                if(poly_cgal.is_clockwise_oriented())
                    poly_cgal.reverse_orientation();
                
                /*for(int i = 0;i < poly_cgal.size();i++){
                    PointCgal p = poly_cgal.vertex(i);
                    ROS_INFO_STREAM("p:(" << p.x() << "," << p.y() << ")");
                }*/

                offset_polygons = CGAL::create_interior_skeleton_and_offset_polygons_2(lOffset,poly_cgal);
    //std::cout << __FILE__ << __LINE__ << std::endl;
                //if(offset_polygons.size() > 0)
                    //count_level++;
                //ROS_INFO_STREAM("offset_polygons_size:" << offset_polygons.size());
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
                    
                    /*ROS_INFO_STREAM("node_cell->polygon:" << node_cell->polygon);
                    ROS_INFO_STREAM("node_cell->n_children:" << node_cell->n_children);
                    ROS_INFO_STREAM("node_cell->n_point:" << node_cell->n_point);
                    ROS_INFO_STREAM("node_cell->level:" << node_cell->level);
                    ROS_INFO_STREAM("node_cell->children:" << node_cell->children);*/

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
    //ROS_INFO_STREAM("tempNode_size:" << tempNode.size());
    for(int i = 0;i < tempNode.size();i++){
        //ROS_INFO_STREAM("tempNode_size:" << tempNode.size());
        //geometry_msgs::Polygon partial_polygon;
        //ROS_INFO_STREAM("tempNode_[" << i << "]:" << tempNode[i].size());
        for(int j = 0;j < tempNode[i].size();j++){
            geometry_msgs::Polygon partial_polygon;
            //ROS_INFO_STREAM("tempNode_level:" << tempNode[i][j]->level);
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
    //ROS_INFO_STREAM("iss.size_of_vertices():" << iss->size_of_vertices());
    //ROS_INFO_STREAM("iss.size_of_halfedges():" << iss->size_of_halfedges());
    //ROS_INFO_STREAM("iss.size_of_faces():" << iss->size_of_faces());
    
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
                //ROS_INFO_STREAM("#####################################################");
                //ROS_INFO_STREAM("map.count:" << SSEdgeTop.count(ss_edge));
                
                //ROS_INFO_STREAM("v_is_skeleton:" << v->is_skeleton());
                //ROS_INFO_STREAM("from:(" << from.x << "," << from.y << ")");
                
                //ROS_INFO_STREAM("v_op_is_contour:" << v_op->is_contour());
                //ROS_INFO_STREAM("to:(" << to.x << "," << to.y << ")");

                if(v->is_skeleton() && v_op->is_contour())
                {
                    if(SSEdgeTop.count(ss_edge) == 0){
                        double t = v->time();
                        if(t < 10.0){
                            ss_edge.isSplitNodeFrom = true;
                            ss_edge.time = t;

                            //ROS_INFO_STREAM("v_t:" << t);
                            /*geometry_msgs::Pose p_t;
                            p_t.position.x = v->point().x()*req.map_resolution+req.map_origin_x;
                            p_t.position.y = v->point().y()*req.map_resolution+req.map_origin_y;
                            p_t.position.z = 0.f;

                            ROS_INFO_STREAM("p_t:(" << p_t.position.x << "," << p_t.position.y << ")");
                            res.point_skeleton.push_back(p_t);*/

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
    
    //ROS_INFO_STREAM("$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$");    
    
    //init KDtree
    KDTree<LocalPoint> kdtree(extPoint);
    
    //store spilt edge
    std::map<SPEdge,int>::iterator iter_sp_edge;
    std::map<SPEdge,int,edgeCmp> SPEdgeTop;

    std::map<SSEdge,int>::iterator iter_edge;
    double sub_area_limit = 1.5;
    double split_edge_limit = 0.5;
    int split_key = 0;
    int sp_union_key = 0;

    std::map<SPUnion,int> SPUnionTop;
    std::map<SPUnion,int> SPUnionTop_;
    std::vector<LocalPoint> spUnionPoint;

    //std::map<SPPoint,int> SPPointTop;
    //int sp_point_key = 0;

    //ROS_INFO_STREAM("contour_ext:" << contour_ext.size());
    for(iter_edge = SSEdgeTop.begin();iter_edge != SSEdgeTop.end();iter_edge++){
        LocalPoint query(iter_edge->first.from.x*req.map_resolution+req.map_origin_x,
                         iter_edge->first.from.y*req.map_resolution+req.map_origin_y);
        //ROS_INFO_STREAM("*************************************************************************");
        /*ROS_INFO_STREAM("from:(" << iter_edge->first.from.x*req.map_resolution+req.map_origin_x << ","
                                 << iter_edge->first.from.y*req.map_resolution+req.map_origin_y << ")");*/
        
        const std::vector<int> knnIndices = kdtree.knnSearch(query, 2);
        int start_id = knnIndices[0] < knnIndices[1]?knnIndices[0]:knnIndices[1];
        int end_id = knnIndices[0] > knnIndices[1]?knnIndices[0]:knnIndices[1];
        
        ROS_INFO_STREAM("start_id:" << start_id << ",end_id" << end_id );
        PointCgal segment_point_a(cv::Point2d(extPoint[start_id]).x,cv::Point2d(extPoint[start_id]).y);
        PointCgal segment_point_b(cv::Point2d(extPoint[end_id]).x,cv::Point2d(extPoint[end_id]).y);

        Segment_Cgal segment_ab(segment_point_a,segment_point_b);

        for(int index_ = start_id+1;index_ < end_id-1;index_++){
            PointCgal segment_temp_a(cv::Point2d(extPoint[index_]).x,cv::Point2d(extPoint[index_]).y);
            PointCgal segment_temp_b(cv::Point2d(extPoint[index_+1]).x,cv::Point2d(extPoint[index_+1]).y);

            Segment_Cgal segment_temp_ab(segment_temp_a,segment_temp_b);
            CGAL::cpp11::result_of<Intersect_Cgal(Segment_Cgal, Segment_Cgal)>::type result = intersection(segment_ab, segment_temp_ab);

            if(result){
                double delta_a_x = segment_temp_a.x()-cv::Point2d(query).x;
                double delta_a_y = segment_temp_a.y()-cv::Point2d(query).y;
                double delta_b_x = segment_temp_b.x()-cv::Point2d(query).x;
                double delta_b_y = segment_temp_b.y()-cv::Point2d(query).y;
                
                double distance_a = std::sqrt(delta_a_x*delta_a_x+delta_a_y*delta_a_y);
                double distance_b = std::sqrt(delta_a_x*delta_a_x+delta_a_y*delta_a_y);

                if(distance_a < distance_b){
                    double delta_s_x = segment_temp_a.x()-cv::Point2d(extPoint[start_id]).x;
                    double delta_s_y = segment_temp_a.y()-cv::Point2d(extPoint[start_id]).y;
                    double delta_e_x = segment_temp_a.x()-cv::Point2d(extPoint[end_id]).x;
                    double delta_e_y = segment_temp_a.y()-cv::Point2d(extPoint[end_id]).y;
                
                    double distance_s = std::sqrt(delta_s_x*delta_s_x+delta_s_y*delta_s_y);
                    double distance_e = std::sqrt(delta_e_x*delta_e_x+delta_e_y*delta_e_y);

                    if(distance_s < distance_e)
                    {
                        start_id = index_;
                        break;
                    }
                    else{
                        end_id = index_;
                        break;
                    }
                }
                else{
                    double delta_s_x = segment_temp_b.x()-cv::Point2d(extPoint[start_id]).x;
                    double delta_s_y = segment_temp_b.y()-cv::Point2d(extPoint[start_id]).y;
                    double delta_e_x = segment_temp_b.x()-cv::Point2d(extPoint[end_id]).x;
                    double delta_e_y = segment_temp_b.y()-cv::Point2d(extPoint[end_id]).y;
                
                    double distance_s = std::sqrt(delta_s_x*delta_s_x+delta_s_y*delta_s_y);
                    double distance_e = std::sqrt(delta_e_x*delta_e_x+delta_e_y*delta_e_y);

                    if(distance_s < distance_e)
                    {
                        start_id = index_;
                        break;
                    }
                    else{
                        end_id = index_;
                        break;
                    }

                }
                ROS_INFO_STREAM("intersection!");
            }
            //else
                //ROS_INFO_STREAM("no intersection!");
        }
        
        ROS_INFO_STREAM("start_id*******:" << start_id << ",end_id************:" << end_id);
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

        //ROS_INFO_STREAM("ID:(" << start_id<< "," << end_id << ")");
        //ROS_INFO_STREAM("SPEdgeTop.count(sp_edge):" << SPEdgeTop.count(sp_edge));
        if(SPEdgeTop.count(sp_edge) == 0){
            std::vector<cv::Point2f> sub_contour;
            std::vector<cv::Point2f> sub_contour_;
            std::vector<int> index_contour;
            std::vector<int> index_contour_;
            
            for(int i = 0;i < contour_ext.size();i++){
                if((i == sp_edge.start_id) || (i == sp_edge.end_id)){
                        sub_contour.push_back(cv::Point2f(cv::Point2d(extPoint[i]).x,cv::Point2d(extPoint[i]).y));
                        sub_contour_.push_back(cv::Point2f(cv::Point2d(extPoint[i]).x,cv::Point2d(extPoint[i]).y));

                        index_contour.push_back(i);
                        index_contour_.push_back(i);
                }
                else{
                    if((i > sp_edge.start_id) && (i < sp_edge.end_id)){
                        sub_contour.push_back(cv::Point2f(cv::Point2d(extPoint[i]).x,cv::Point2d(extPoint[i]).y));
                        //cv::Point2f cv_point(cv::Point2d(extPoint[i]).x,cv::Point2d(extPoint[i]).y);
                        //SPPoint sp_point(i);
                        //SPPointTop.insert(std::make_pair(sp_point,sp_point_key++));
                        index_contour.push_back(i);
                    }
                    else{
                        sub_contour_.push_back(cv::Point2f(cv::Point2d(extPoint[i]).x,cv::Point2d(extPoint[i]).y));
                        index_contour_.push_back(i);
                    }
                }
            }

            //ROS_INFO_STREAM("sub_contour:" << sub_contour.size());
            //ROS_INFO_STREAM("sub_contour_:" << sub_contour_.size());

            if(sub_contour.size() < 3 || sub_contour_.size() < 3)
                continue;
            
            double sub_area = cv::contourArea(sub_contour,false);
            double sub_area_ = cv::contourArea(sub_contour_,false);
            //ROS_INFO_STREAM("sub_area:" << sub_area);
            //ROS_INFO_STREAM("sub_area_:" << sub_area_);

            if(sub_area > sub_area_limit && sub_area_ > sub_area_limit){
                double delta_x = cv::Point2d(extPoint[sp_edge.start_id]).x-cv::Point2d(extPoint[sp_edge.end_id]).x;
                double delta_y = cv::Point2d(extPoint[sp_edge.start_id]).y-cv::Point2d(extPoint[sp_edge.end_id]).y;
                //ROS_INFO_STREAM("split_edge_limit:" << std::sqrt(delta_x*delta_x + delta_y*delta_y));

                if(std::sqrt(delta_x*delta_x + delta_y*delta_y) > split_edge_limit){
                    SPEdgeTop.insert(std::make_pair(sp_edge,split_key++));      

                    /*ROS_INFO_STREAM("kdtree_start:(" << cv::Point2d(extPoint[sp_edge.start_id]).x << "," 
                                               << cv::Point2d(extPoint[sp_edge.start_id]).y << ")");*/
                    geometry_msgs::Pose p_start;
                    p_start.position.x = cv::Point2d(extPoint[sp_edge.start_id]).x;
                    p_start.position.y = cv::Point2d(extPoint[sp_edge.start_id]).y;
                    p_start.position.z = 0.f;
                    
                    //res.point_nearest.push_back(p_start);

                    /*ROS_INFO_STREAM("kdtree_end:(" << cv::Point2d(extPoint[sp_edge.end_id]).x << "," 
                                               << cv::Point2d(extPoint[sp_edge.end_id]).y << ")");*/
                    geometry_msgs::Pose p_end;
                    p_end.position.x = cv::Point2d(extPoint[sp_edge.end_id]).x;
                    p_end.position.y = cv::Point2d(extPoint[sp_edge.end_id]).y;
                    p_end.position.z = 0.f;
                    
                    //res.point_nearest.push_back(p_end);
                    
                    geometry_msgs::Pose p_s;
                    p_s.position.x = iter_edge->first.from.x*req.map_resolution+req.map_origin_x;
                    p_s.position.y = iter_edge->first.from.y*req.map_resolution+req.map_origin_y;
                    p_s.position.z = 0.f;
                    
                    //res.point_skeleton.push_back(p_s);
                    
                    cv::Point2f p_s_(p_s.position.x,p_s.position.y);
                    cv::Point2f p_start_(p_start.position.x,p_start.position.y);
                    cv::Point2f p_end_(p_end.position.x,p_end.position.y);
                    //SPUnion sp_union(p_s_,p_start_,p_end_);
                    
                    if(sub_area < sub_area_){
                        std::vector<int> index_contour_sort;
                        int s_index = 0;
                        std::stack<int> sIndex;
                        
                        /*for(int i = 0;i < index_contour.size();i++){
                            ROS_INFO_STREAM("index_contour:" << index_contour[i]);
                        }*/

                        if((index_contour[0] != sp_edge.start_id) || (index_contour[index_contour.size()-1] != sp_edge.end_id)){
                           for(int i = 0;i < index_contour.size();i++){
                                if(index_contour[i] == sp_edge.start_id){
                                    s_index = i;
                                    break;
                                }
                           }
                            for(int i = s_index+1;i < index_contour.size();i++)
                                sIndex.push(index_contour[i]);

                            for(int i = 0;i <= s_index;i++)
                                sIndex.push(index_contour[i]);

                            while(!sIndex.empty()){
                                int s_index_ = sIndex.top();
                                index_contour_sort.push_back(s_index_);
                                sIndex.pop();
                            }
                        }
                        else{
                            for(int i = 0;i < index_contour.size();i++)
                                index_contour_sort.push_back(index_contour[i]);
                        }
                            
                         
                        /*for(int i = s_index+1;i < index_contour.size();i++)
                            sIndex.push(index_contour[i]);

                        for(int i = 0;i <= s_index;i++)
                            sIndex.push(index_contour[i]);

                        while(!sIndex.empty()){
                            int s_index_ = sIndex.top();
                            index_contour_sort.push_back(s_index_);
                            sIndex.pop();
                        }*/

                        SPUnion sp_union(p_s_,p_start_,sp_edge.start_id,p_end_,sp_edge.end_id,index_contour_sort);
                        SPUnionTop.insert(std::make_pair(sp_union,sp_union_key++));

                        
                        /*for(int i = 0;i < index_contour_sort.size();i++){
                            ROS_INFO_STREAM("index_contour_sort:" << index_contour_sort[i]);
                            //SPPoint sp_point(index_contour[i]);
                            //SPPointTop.insert(std::make_pair(sp_point,sp_point_key++));
                        }*/
                    }
                    else{
                        std::vector<int> index_contour_sort_;
                        int s_index = 0;
                        std::stack<int> sIndex;
                        
                        /*for(int i = 0;i < index_contour_.size();i++){
                            ROS_INFO_STREAM("index_contour_:" << index_contour_[i]);
                        }*/

                        if((index_contour_[0] != sp_edge.start_id) || (index_contour_[index_contour.size()-1] != sp_edge.end_id)){
                           for(int i = 0;i < index_contour_.size();i++){
                                if(index_contour_[i] == sp_edge.start_id){
                                    s_index = i;
                                    break;
                                }
                           }

                            for(int i = s_index+1;i < index_contour_.size();i++)
                                sIndex.push(index_contour_[i]);

                            for(int i = 0;i <= s_index;i++)
                                sIndex.push(index_contour_[i]);

                            while(!sIndex.empty()){
                                int s_index_ = sIndex.top();
                                index_contour_sort_.push_back(s_index_);
                                sIndex.pop();
                            }
                        }
                        else{
                            for(int i = 0;i < index_contour_.size();i++)
                                index_contour_sort_.push_back(index_contour_[i]);
                        }
                         
                        /*for(int i = s_index+1;i < index_contour_.size();i++)
                            sIndex.push(index_contour_[i]);

                        for(int i = 0;i <= s_index;i++)
                            sIndex.push(index_contour_[i]);

                        while(!sIndex.empty()){
                            int s_index_ = sIndex.top();
                            index_contour_sort_.push_back(s_index_);
                            sIndex.pop();
                        }*/
                        
                        
                        SPUnion sp_union(p_s_,p_start_,sp_edge.end_id,p_end_,sp_edge.start_id,index_contour_sort_);
                        SPUnionTop.insert(std::make_pair(sp_union,sp_union_key++));
                        /*for(int i = 0;i < index_contour_sort_.size();i++){
                            ROS_INFO_STREAM("index_contour_sort_:" << index_contour_sort_[i]);
                            //SPPoint sp_point(index_contour_[i]);
                            //SPPointTop.insert(std::make_pair(sp_point,sp_point_key++));
                        }*/
                    }

                    spUnionPoint.push_back(LocalPoint(p_s.position.x,p_s.position.y));
                }
            }
        }

        /*ROS_INFO_STREAM("to:(" << iter_edge->first.to.x*req.map_resolution+req.map_origin_x << ","
                               << iter_edge->first.to.y*req.map_resolution+req.map_origin_y << ")");
        ROS_INFO_STREAM("isSplitNodeFrom:" << iter_edge->first.isSplitNodeFrom << ",isSplitNodeTo:" << iter_edge->first.isSplitNodeTo);
        ROS_INFO_STREAM("time:" << iter_edge->first.time);
        ROS_INFO_STREAM("$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$");*/


    }
   
    ROS_INFO_STREAM("spUnionPoint_size:" << spUnionPoint.size());
    std::map<SPUnion,int>::iterator iter_union; 

    KDTree<LocalPoint> kdtree_union(spUnionPoint);
    double radius = 1.5;
    int delete_count = 0;
    std::map<int,int> SPUnionTop_sel;
    int SPUnionTop_sel_key = 0;

    ROS_INFO_STREAM("SPUnionTop_size:" << SPUnionTop.size());
    for(iter_union = SPUnionTop.begin();iter_union != SPUnionTop.end();iter_union++){
        
        LocalPoint query(iter_union->first.p.x,iter_union->first.p.y);
        
        ROS_INFO_STREAM("query:(" << iter_union->first.p.x << "," << iter_union->first.p.y << ")");
        const std::vector<int> radIndices = kdtree_union.radiusSearch(query, radius);
        ROS_INFO_STREAM("radIndices_size:" << radIndices.size());
        if(radIndices.size() > 1){
            /*if((radIndices.size()-delete_count) > 1){
                SPUnionTop.erase(iter_union++);
                delete_count++;
            }
            else
                iter_union++;*/
            bool sel_flag = true;

            for(int i = 0;i < radIndices.size();i++){
                if(SPUnionTop_sel.count(radIndices[i]) == 0)
                    SPUnionTop_sel.insert(std::make_pair(radIndices[i],SPUnionTop_sel_key++));
                else
                    sel_flag &= false;
            }
            
            if(sel_flag)
                SPUnionTop_.insert(*iter_union);
                
         }
        else{
            //iter_union++;
            ROS_INFO_STREAM("query*****:(" << iter_union->first.p.x << "," << iter_union->first.p.y << ")");
            SPUnionTop_.insert(*iter_union);
        }
    }

    ROS_INFO_STREAM("SPUnionTop_size_:" << SPUnionTop_.size());
    
    std::vector<SPEdge> s_e;
    for(iter_union = SPUnionTop_.begin();iter_union != SPUnionTop_.end();iter_union++){
        PolygonCgal poly_cgal_union;

        geometry_msgs::Pose p_start;
        p_start.position.x = iter_union->first.c1.x;
        p_start.position.y = iter_union->first.c1.y;
        p_start.position.z = 0.f;
        
        res.point_nearest.push_back(p_start);

        geometry_msgs::Pose p_end;
        p_end.position.x = iter_union->first.c2.x;
        p_end.position.y = iter_union->first.c2.y;
        p_end.position.z = 0.f;
        
        res.point_nearest.push_back(p_end);
        
        geometry_msgs::Pose p_s;
        p_s.position.x = iter_union->first.p.x;
        p_s.position.y = iter_union->first.p.y;
        p_s.position.z = 0.f;
        
        res.point_skeleton.push_back(p_s);

        SPEdge e(iter_union->first.c1_index,iter_union->first.c2_index);
        s_e.push_back(e);

    }
    
    ROS_INFO_STREAM("s_e_size:" << s_e.size());
    
    int count_e = 0;
    //std::map<int, std::vector<std::tuple<int, int,double>>> EWD;
    int ewd_key = 0;
    for(int i = 0;i < contour_ext.size();i++){
        if(i == (contour_ext.size()-1)){
            SPEdge e(0,0);
            if(IsRoot(i,s_e,e)){
                if(e.start_id == i)
                    EWD[i].push_back(std::make_tuple(e.start_id,e.end_id,1));
                else
                    EWD[i].push_back(std::make_tuple(e.end_id,e.start_id,1));
            }
            
            EWD[i].push_back(std::make_tuple(contour_ext.size()-1,0,1));
                
        }
        else{
            SPEdge e(0,0);
            
            if(IsRoot(i,s_e,e)){
                if(e.start_id == i)
                    EWD[i].push_back(std::make_tuple(e.start_id,e.end_id,1));
                else
                    EWD[i].push_back(std::make_tuple(e.end_id,e.start_id,1));
            }


            EWD[i].push_back(std::make_tuple(i,i+1,1));
        }
    }
    
    EWD[0].push_back(std::make_tuple(0,contour_ext.size()-1,1));
    
    ROS_INFO_STREAM("EWD_size:" << EWD.size());
    ROS_INFO_STREAM("count_e:" << count_e);
    ROS_INFO_STREAM("Digraph:");

    for(int v = 0;v < EWD.size();v++){
        std::cout << v << ":";
        for(std::vector<std::tuple<int, int, double>>::iterator ii = EWD[v].begin(); ii != EWD[v].end(); ii++)
            std::cout << std::get<0>(*ii) << "->" << std::get<1>(*ii) << " " << std::get<2>(*ii) << "  ";

        std::cout << std::endl;
    }
    
    /*marked = new bool[EWD.size()];
    for(int i = 0;i < EWD.size();i++)
        marked[i] = false;

    onStack = new bool[EWD.size()];
    for(int i = 0;i < EWD.size();i++)
        onStack[i] = false;

    edgeTo = new std::tuple<int , int , double>[EWD.size()];
    cycle = new std::stack<std::tuple<int , int , double>>[EWD.size()];

    for(int v = 0;v < EWD.size();v++){
        if(!marked[v])
            dfsCycle(v);
    }

    ROS_INFO_STREAM("n_cycle:" << n_cycle);
    
    for(int i = 0;i < n_cycle;i++){
        ROS_INFO_STREAM("************************");
        while(!cycle[i].empty()){
            std::tuple<int,int,double> s = cycle[i].top();
            std::cout << std::get<0>(s) << "->" << std::get<1>(s) << " " << std::get<2>(s) << "  " << std::endl;
            cycle[i].pop();
        }
    }

    delete[] marked;
    delete[] onStack;
    delete[] edgeTo;
    delete[] cycle;*/
    
    //std::map<int,int> mapContourExt;
    SPEdge e_l(0,contour_ext.size()-1);
    s_e.push_back(e_l);

    int key_map_ext = 0;
    for(int e = 0;e < s_e.size();e++){
        ROS_INFO_STREAM("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&");
        visited = new bool[EWD.size()];
        for(int i = 0;i < EWD.size();i++)
            visited[i] = false;

        DfsFindPaths(s_e[e].start_id,s_e[e].end_id);

        ROS_INFO_STREAM("DPaths_size:" << DPaths.size());
        int indexDPaths = FindSplitPath(DPaths);
        
        geometry_msgs::Polygon partial_polygon;
        while(!DPaths[indexDPaths].empty()){

            int p_i = DPaths[indexDPaths].top();
            //ROS_INFO_STREAM("DPaths:" << p_i);
            //if(mapContourExt.count(p_i) == 0)
                //mapContourExt.insert(std::make_pair(p_i,key_map_ext++));

            geometry_msgs::Point32 point;
            point.x = contour_ext[p_i].x*req.map_resolution+req.map_origin_x;
            point.y = contour_ext[p_i].y*req.map_resolution+req.map_origin_y;
            partial_polygon.points.push_back(point);
            
            DPaths[indexDPaths].pop();
        }
        
        res.polygon_test.push_back(partial_polygon); 
        
        delete[] visited;
        DPaths.clear();
    }

    
    /*geometry_msgs::Polygon partialPolygon;
    for(int i = 0;i < contour_ext.size();i++){
        if(mapContourExt.count(i) == 0){
            geometry_msgs::Point32 point;
            point.x = contour_ext[i].x*req.map_resolution+req.map_origin_x;
            point.y = contour_ext[i].y*req.map_resolution+req.map_origin_y;
            partialPolygon.points.push_back(point);
        }
    }
    
    res.polygon_test.push_back(partialPolygon);*/

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

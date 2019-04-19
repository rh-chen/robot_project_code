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
#include<algorithm>
#include<CGAL/create_offset_polygons_2.h>
#include <stdio.h>
#include <stdlib.h>


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

typedef struct stack_t
{
    NODE** array;
    int    index;
    int    size;
} STACK;

typedef struct queue_t
{
    NODE** array;
    int    head;
    int    tail;
    int    num;
    int    size;
} QUEUE;

void* util_malloc(int size)
{
    void* ptr = malloc(size);

    if (ptr == NULL)
    {
        printf("Memory allocation error!\n");
        exit(EXIT_FAILURE);
    }

    return ptr;
}

char* util_strdup(char* src)
{
    char* dst = strdup(src);

    if (dst == NULL)
    {
        printf ("Memroy allocation error!\n");
        exit(EXIT_FAILURE);
    }

    return dst;
}

FILE* util_fopen(char* name, char* access)
{
    FILE* fp = fopen(name, access);

    if (fp == NULL)
    {
        printf("Error opening file %s!\n", name);
        exit(EXIT_FAILURE);
    }

    return  fp;
}


STACK* STACKinit(int size)
{
    STACK* sp;

    sp = (STACK*)util_malloc(sizeof (STACK));
    sp->size  = size;
    sp->index = 0;
    sp->array = (NODE**)util_malloc(size * sizeof (NODE*));

    return sp;
}

int STACKempty(STACK* sp)
{
    if (sp == NULL || sp->index <= 0)
    {
        return 1;
    }
    return 0;
}

int STACKpush(STACK* sp, NODE* data)
{
    if (sp == NULL || sp->index >= sp->size)
    {
        return 0;
    }

    sp->array[sp->index++] = data;
    return 1;
}

int STACKpop(STACK* sp, NODE** data_ptr)
{
    if (sp == NULL || sp->index <= 0)
    {
        return 0;
    }

    *data_ptr = sp->array[--sp->index];
    return 1;
}

void STACKdestroy(STACK* sp)
{
    free(sp->array);
    free(sp);
}

QUEUE* QUEUEinit(int size)
{
    QUEUE* qp;

    qp = (QUEUE*)util_malloc(sizeof (QUEUE));
    qp->size  = size;
    qp->head  = qp->tail = qp->num = 0;
    qp->array = (NODE**)util_malloc(size * sizeof (NODE*));

    return qp;
}

int QUEUEenqueue(QUEUE* qp, NODE* data)
{
    if (qp == NULL || qp->num >= qp->size)
    {
        return 0;
    }

    qp->array[qp->tail] = data;
    qp->tail = (qp->tail + 1) % (qp->size);
    ++qp->num;
    return 1;
}

int QUEUEdequeue(QUEUE* qp, NODE** data_ptr)
{
    if (qp == NULL || qp->num <= 0)
    {
        return 0;
    }

    *data_ptr = qp->array[qp->head];
    qp->head = (qp->head + 1) % (qp->size);
    --qp->num;

    return 1;
}

int QUEUEempty(QUEUE* qp)
{
    if (qp == NULL || qp->num <= 0)
    {
        return 1;
    }

    return 0;
}

void QUEUEdestroy(QUEUE* qp)
{
    free(qp->array);
    free(qp);
}

/*NODE* create_node()
{
    NODE* q;

    q = (NODE*)util_malloc(sizeof (NODE));
    q->n_children = 0;
    q->level      = 0;
    q->children   = NULL;

    return q;
}*/

/*NODE* search_node_r(char* name, NODE* head)
{
    NODE* temp = NULL;
    int i = 0;

    if (head != NULL)
    {
        if (strcmp(name, head->name) == 0)
        {
            temp = head;
        }
        else
        {
            for (i = 0; i < head->n_children && temp == NULL; ++i)
            {
                temp = search_node_r(name, head->children[i]);
            }
        }
    }

    return temp;
}*/

/*void read_file(NODE** head, char* filename)
{
    NODE* temp = NULL;
    int i = 0, n = 0;
    char name[M], child[M];
    FILE* fp;

    fp = util_fopen(filename, "r");

    while (fscanf(fp, "%s %d", name, &n) != EOF)
    {
        if (*head == NULL)
        {
            temp = *head = create_node();
            temp->name = util_strdup(name);
        }
        else
        {
            temp = search_node_r(name, *head);
        }
        temp->n_children = n;
        temp->children   = (NODE**)malloc(n * sizeof (NODE*));
        if (temp->children == NULL)
        {
            fprintf(stderr, "Dynamic allocation error!\n");
            exit(EXIT_FAILURE);
        }

        for (i = 0; i < n; ++i)
        {
            fscanf(fp, "%s", child);
            temp->children[i] = create_node();
            temp->children[i]->name = util_strdup(child);
        }
    }

    fclose(fp);
}*/

void f1(NODE* head)
{
    NODE* p = NULL;
    QUEUE* q = NULL;
    STACK* s = NULL;
    int i = 0;

    q = QUEUEinit(100);
    s = STACKinit(100);

    head->level = 0;
    
    QUEUEenqueue(q, head);
    
    while (QUEUEempty(q) == 0)
    {
        QUEUEdequeue(q, &p);
        for (i = 0; i < p->n_children; ++i)
        {
            p->children[i]->level = p->level + 1;
            QUEUEenqueue(q, p->children[i]);
        }
        STACKpush(s, p);
    }

    while (STACKempty(s) == 0)
    {
        STACKpop(s, &p);
        //fprintf(stdout, "   %d %s\n", p->level, p->name);
    }

    QUEUEdestroy(q);
    STACKdestroy(s);
}

/*void f2(NODE* head, char* str, char** strBest, int level)
{
    int   i   = 0;
    char* tmp = NULL;

    if (head == NULL)
    {
        return;
    }

    tmp = (char*)util_malloc((strlen(str) + strlen(head->name) + 1) * sizeof (char));
    sprintf(tmp, "%s%s", str, head->name);

    if (head->n_children == 0)
    {
        if (*strBest == NULL || strlen(tmp) > strlen(*strBest))
        {
            free(*strBest);
            *strBest = util_strdup(tmp);
        }
    }

    for (i = 0; i < head->n_children; ++i)
    {
        f2(head->children[i], tmp, strBest, level + 1);
    }

    free(tmp);
}*/

void free_tree_r(NODE* head)
{
    int i = 0;
    if (head == NULL)
    {
        return;
    }

    for (i = 0; i < head->n_children; ++i)
    {
        free_tree_r(head->children[i]);
    }

    //free(head->name);
    free(head);
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

bool isBadPolygon(PointVector& polygon_,double step_){
    
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
}
void selectPolygon(PolygonListCgal& src_,PolygonListCgal& dst_)
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

}

/*typedef struct skeletonPoint{
    int x;
    int y;

    skeletonPoint(int x_,int y_){
        x = x_;
        y = y_;
    }

    bool operator < (const skeletonPoint& p)const
    {
        if(x*y == p.x*p.y)
            return (x < p.x?true:false);
        else
            return (x*y < p.x*p.y?true:false);
    }

}SP;*/

void PolygonCgalToPtr(cv::Point2f* ptr,int n,PolygonCgal& poly){
    for(int i = 0;i < n;i++){
        poly.push_back(PointCgal(ptr[i].x,ptr[i].y));
    }
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
    cv::approxPolyDP(vertices_point,contour_ext,2.0,true);
    
    //B+Tree
    NODE* head = NULL;
    head = new NODE(contour_ext.size(),0,0);

    ROS_INFO_STREAM("head->polygon:" << head->polygon);
    ROS_INFO_STREAM("head->n_children:" << head->n_children);
    ROS_INFO_STREAM("head->n_point:" << head->n_point);
    ROS_INFO_STREAM("head->level:" << head->level);
    ROS_INFO_STREAM("head->children:" << head->children);

    //PolygonCgal poly_cgal;
	for(int j = 0;j < contour_ext.size();j++){
		double point_x = contour_ext[j].x*req.map_resolution+req.map_origin_x;
		double point_y = contour_ext[j].y*req.map_resolution+req.map_origin_y;
				
		//int point_x = vertices_point[j].x;
		//int point_y = vertices_point[j].y;
		
        geometry_msgs::Pose pose_;
		pose_.position.x = point_x;
		pose_.position.y = point_y;

		//res.pose.push_back(pose_);	
        //poly_cgal.push_back(PointCgal(point_x,point_y));
    //std::cout << __FILE__ << __LINE__ << std::endl;
        head->polygon[j].x = point_x;
        head->polygon[j].y = point_y;
    //std::cout << __FILE__ << __LINE__ << std::endl;
	}
    
    //std::cout << __FILE__ << __LINE__ << std::endl;
    //if(poly_cgal.is_clockwise_oriented())
        //poly_cgal.reverse_orientation();

    //std::cout << __FILE__ << __LINE__ << std::endl;
    for(int i = 0;i < head->n_children;i++){
       ROS_INFO_STREAM("head->polygon:(" << head->polygon[i].x << "," << head->polygon[i].y << ")");
    }
    //PolygonWithHolesCgal polyHoles(poly_cgal);
    //polyHoles.outer_boundary() = poly_cgal;
    
    //SsPtr iss = CGAL::create_interior_straight_skeleton_2(polyHoles);
    //ROS_INFO_STREAM("iss.size_of_vertices():" << iss->size_of_vertices());
    //ROS_INFO_STREAM("iss.size_of_halfedges():" << iss->size_of_halfedges());
    //ROS_INFO_STREAM("iss.size_of_faces():" << iss->size_of_faces());
    
    //std::cout << __FILE__ << __LINE__ << std::endl;
    double lOffset = 0.2;
    //PolygonPtrVector offset_polygons = CGAL::create_offset_polygons_2<PolygonCgal>(lOffset,*iss);
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

    ROS_INFO_STREAM("tempNode_size:" << tempNode.size());
    for(int i = 0;i < tempNode.size();i++){
        //ROS_INFO_STREAM("tempNode_size:" << tempNode.size());
        //geometry_msgs::Polygon partial_polygon;
        ROS_INFO_STREAM("tempNode_[" << i << "]:" << tempNode[i].size());
        for(int j = 0;j < tempNode[i].size();j++){
            geometry_msgs::Polygon partial_polygon;

            for(int k = 0;k < tempNode[i][j]->n_point;k++){
                geometry_msgs::Point32 point_32;
                point_32.x = tempNode[i][j]->polygon[k].x;
                point_32.y = tempNode[i][j]->polygon[k].y;

                partial_polygon.points.push_back(point_32);
            }
            res.polygon.push_back(partial_polygon);
        }
    }
    /*std::map<SP,int> skeletonPointMap;
    int key_ = 0;
    if(iss){
        for(auto face = iss->faces_begin();face != iss->faces_end();face++){
            Ss::Halfedge_const_handle begin = face->halfedge();
            Ss::Halfedge_const_handle edge = begin;

            do{
                const Vertex_const_handle& v = edge->vertex();
                
                SP sp_(v->point().x(),v->point().y());
                ROS_INFO_STREAM("map.count:" << skeletonPointMap.count(sp_));
                ROS_INFO_STREAM("is_skeleton:" << v->is_skeleton());
                ROS_INFO_STREAM("sp_:(" << sp_.x << "," << sp_.y << ")");
                
                if(v->is_skeleton())
                {
                    if(skeletonPointMap.count(sp_) != 1){
                        geometry_msgs::Pose p_t;
                        p_t.position.x = v->point().x()*req.map_resolution+req.map_origin_x;;
                        p_t.position.y = v->point().y()*req.map_resolution+req.map_origin_y;;
                        p_t.position.z = 0.f;

                        res.point_skeleton.push_back(p_t);

                        skeletonPointMap.insert(std::make_pair(sp_,key_++));
                    }
                }
                
                edge = edge->prev();

            }while(edge != begin);
        }
    }
    
    ROS_INFO_STREAM("key_:" << key_);
    */

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

    /*for(int index_i = 0;index_i < final_path.size();index_i++){
		for(int index_j = 0;index_j < final_path[index_i].size();index_j++){
			geometry_msgs::PoseStamped current_pose;
			current_pose.pose.position.x = final_path[index_i][index_j].x;
			current_pose.pose.position.y = final_path[index_i][index_j].y;
			current_pose.pose.position.z = final_path[index_i][index_j].z;

			plan_path.poses.push_back(current_pose);
		}
	}

    res.path.push_back(plan_path);*/

    /*for(int index_i = 0;index_i < final_path.size();index_i++){
		nav_msgs::Path path_bcd;

		for(int index_j = 0;index_j < final_path[index_i].size();index_j++){
			geometry_msgs::PoseStamped current_pose;
			current_pose.pose.position.x = final_path[index_i][index_j].x;
			current_pose.pose.position.y = final_path[index_i][index_j].y;
			current_pose.pose.position.z = final_path[index_i][index_j].z;

			path_bcd.poses.push_back(current_pose);
		}
		res.path.push_back(path_bcd);
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

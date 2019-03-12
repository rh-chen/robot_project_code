#define CGAL_PARTITION_BRUTE_FORCE_FIX

#include <vector>
#include <boost/shared_ptr.hpp>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_with_holes_2.h>
#include <CGAL/Boolean_set_operations_2.h>
#include <CGAL/Polygon_vertical_decomposition_2.h>

typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef Kernel::FT Ft;
typedef Kernel::Point_2 Point;
typedef Kernel::Segment_2 Segment;
typedef Kernel::Direction_2 Direction;
typedef Kernel::Line_2 Line;
typedef Kernel::Vector_2 Vector;
typedef CGAL::Polygon_2<Kernel> Polygon;
typedef CGAL::Polygon_with_holes_2<Kernel> PolygonWithHoles;
typedef std::list<Polygon> Polygon_list;
typedef std::vector<Kernel::Point_2> container;

int main() {
 Polygon poly;

 float scale = 4.0/100;
 float max_y = 500*scale;
 
 poly.push_back(Point(76*scale, max_y-496*scale));
 poly.push_back(Point(660*scale, max_y-496*scale));
 poly.push_back(Point(660*scale, max_y-48*scale));
 poly.push_back(Point(71*scale, max_y-54*scale));
 
 Polygon holes[10];
 holes[0].push_back(Point(131*scale, max_y-86*scale));
 holes[0].push_back(Point(179*scale, max_y-85*scale));
 holes[0].push_back(Point(180*scale, max_y-238*scale));
 holes[0].push_back(Point(133*scale, max_y-239*scale));

 holes[1].push_back(Point(237*scale, max_y-84*scale));
 holes[1].push_back(Point(286*scale, max_y-84*scale));
 holes[1].push_back(Point(288*scale, max_y-237*scale));
 holes[1].push_back(Point(240*scale, max_y-238*scale));
 
 holes[2].push_back(Point(345*scale, max_y-84*scale));
 holes[2].push_back(Point(393*scale, max_y-83*scale));
 holes[2].push_back(Point(396*scale, max_y-236*scale));
 holes[2].push_back(Point(348*scale, max_y-236*scale));
 
 
 PolygonWithHoles polyHoles(poly);
 polyHoles.outer_boundary() = poly;

 for (int i=0; i<3; ++i)
    polyHoles.add_hole(holes[i]);
 
 Polygon_list partition_polys;
 CGAL::Polygon_vertical_decomposition_2<Kernel,container> obj;
 obj(polyHoles,std::back_inserter(partition_polys));
 
 std::cout << "partition_polys_size: " << partition_polys.size() << std::endl;
 return 0;
}

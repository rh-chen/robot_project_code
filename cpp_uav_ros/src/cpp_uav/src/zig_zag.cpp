#define CGAL_PARTITION_BRUTE_FORCE_FIX

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Partition_traits_2.h>
#include <CGAL/partition_2.h>
#include <CGAL/point_generators_2.h>
#include <CGAL/random_polygon_2.h>

#include <fstream>
#include <iostream>
#include <sstream>

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Partition_traits_2<K> Traits;
typedef Traits::Point_2 Point_2;
typedef Traits::Polygon_2 Polygon_2;
typedef Polygon_2::Vertex_iterator Vertex_iterator;
typedef std::list<Polygon_2> Polygon_list;
typedef CGAL::Creator_uniform_2<int, Point_2> Creator;
typedef CGAL::Random_points_in_square_2< Point_2, Creator > Point_generator;

void make_polygon_1(Polygon_2& polygon)
{
  polygon.push_back(Point_2(1.4, -4.2));
  polygon.push_back(Point_2(1.4, -3.4));
  polygon.push_back(Point_2(-0.2, -3.4));
  polygon.push_back(Point_2(-0.2, 7.8));
  polygon.push_back(Point_2(3, 7.8));
  polygon.push_back(Point_2(3, 5.4));
  polygon.push_back(Point_2(2.2, 5.4));
  polygon.push_back(Point_2(2.2, 3));
  polygon.push_back(Point_2(3.8, 3));
  polygon.push_back(Point_2(3.8, -0.2));
  polygon.push_back(Point_2(2.2, -0.2));
  polygon.push_back(Point_2(2.2, -1.0));
  polygon.push_back(Point_2(3, -1.0));
  polygon.push_back(Point_2(3, -1.8));
  polygon.push_back(Point_2(3.8, -1.8));
  polygon.push_back(Point_2(3.8, -2.6));
  polygon.push_back(Point_2(4.6, -2.6));
  polygon.push_back(Point_2(4.6, -3.4));
  polygon.push_back(Point_2(3.8, -3.4));
  polygon.push_back(Point_2(3.8, -4.2));
}

void make_polygon_2(Polygon_2& polygon)
{
  polygon.push_back(Point_2(64, 8));
  polygon.push_back(Point_2(64, 16));
  polygon.push_back(Point_2(48, 16));
  polygon.push_back(Point_2(48, 128));
  polygon.push_back(Point_2(80, 128));
  polygon.push_back(Point_2(80, 104));
  polygon.push_back(Point_2(72, 104));
  polygon.push_back(Point_2(72, 80));
  polygon.push_back(Point_2(88, 80));
  polygon.push_back(Point_2(88, 48));
  polygon.push_back(Point_2(72, 48));
  polygon.push_back(Point_2(72, 40));
  polygon.push_back(Point_2(80, 40));
  polygon.push_back(Point_2(80, 32));
  polygon.push_back(Point_2(88, 32));
  polygon.push_back(Point_2(88, 24));
  polygon.push_back(Point_2(96, 24));
  polygon.push_back(Point_2(96, 16));
  polygon.push_back(Point_2(88, 16));
  polygon.push_back(Point_2(88, 8));
}

void make_polygon_3(Polygon_2& polygon)
{
  polygon.push_back(Point_2(391, 374));
  polygon.push_back(Point_2(240, 431));
  polygon.push_back(Point_2(252, 340));
  polygon.push_back(Point_2(374, 320));
  polygon.push_back(Point_2(289, 214));
  polygon.push_back(Point_2(134, 390));
  polygon.push_back(Point_2( 68, 186));
  polygon.push_back(Point_2(154, 259));
  polygon.push_back(Point_2(161, 107));
  polygon.push_back(Point_2(435, 108));
  polygon.push_back(Point_2(208, 148));
  polygon.push_back(Point_2(295, 160));
  polygon.push_back(Point_2(421, 212));
  polygon.push_back(Point_2(441, 303));
}

int main()
{
  Polygon_2 polygon;
  Polygon_list partition_polys;
  Traits partition_traits;

  make_polygon_2(polygon);

  if(polygon.is_clockwise_oriented())
    polygon.reverse_orientation();

  std::ofstream out("input_polygon.cgal");

  out << polygon.size()+1 << "\n";
  for(const auto& pt : polygon.container())
    out << pt << " 0\n";
  out << polygon.container().front() << " 0\n";

  CGAL::optimal_convex_partition_2(polygon.vertices_begin(),
                                   polygon.vertices_end(),
                                   std::back_inserter(partition_polys),
                                   partition_traits);


  assert(CGAL::convex_partition_is_valid_2(polygon.vertices_begin(),
                                           polygon.vertices_end(),
                                           partition_polys.begin(),
                                           partition_polys.end(),
                                           partition_traits));

  std::cout << "partition_polys_size:" << partition_polys.size() << std::endl;
  /*int id = 0;
  for(const Polygon_2& pl : partition_polys)
  {
    std::stringstream oss;
    oss << "partition_polygon_" << id++ << ".cgal" << std::ends;
    std::ofstream out(oss.str().c_str());

    out << pl.size()+1 << "\n";
    for(const auto& pt : pl.container())
      out << pt << " 0\n";
    out << pl.container().front() << " 0\n";
  }*/

  return 0;
}

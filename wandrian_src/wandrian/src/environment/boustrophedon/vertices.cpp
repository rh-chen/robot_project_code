/*
 * vertices.cpp
 *
 *  Created on: Mar 25, 2016
 *      Author: cslab
 */

#include "../../../include/environment/boustrophedon/vertices.hpp"

namespace wandrian {
namespace environment {
namespace boustrophedon {

Vertices::Vertices(PointPtr positions, RectanglePtr polygon) {
  this->position = positions;
  this->polygon = polygon;
}

bool Vertices::left_compared_center() {
  if (this->position->x < this->polygon->get_center()->x)
    return true;
  return false;
}

bool Vertices::upon_compared_center() {
  if (this->position->y > this->polygon->get_center()->y)
    return true;
  return false;
}

bool Vertices::compare_vertices(boost::shared_ptr<Vertices> vertices1, boost::shared_ptr<Vertices> vertices2) {
  if ((vertices1->get_position()->x == vertices2->get_position()->x) ||
      (vertices1->get_position()->y == vertices2->get_position()->y))
    return true;
  return false;
}

bool Vertices::compare_positions_x(boost::shared_ptr<Vertices> vertices1, boost::shared_ptr<Vertices> vertices2) {
  if (vertices1->get_position()->x < vertices2->get_position()->x)
    return true;
  return false;
}

bool Vertices::compare_positions_y(boost::shared_ptr<Vertices> vertices1, boost::shared_ptr<Vertices> vertices2) {
  if (vertices1->get_position()->y < vertices2->get_position()->y)
    return true;
  return false;
}

PointPtr Vertices::get_position() { return position; }

RectanglePtr Vertices::get_polygon() { return polygon; }

void Vertices::set_positions(PointPtr positions) { this->position = positions; }

void Vertices::set_polygon(RectanglePtr polygon) { this->polygon = polygon; }
}
}
}

/*
 * partially_occupiable_identifiable_cell.cpp
 *
 *  Created on: Mar 11, 2016
 *      Author: cslab
 */

#include "../../../include/environment/mstc/partially_occupiable_identifiable_cell.hpp"

namespace wandrian {
namespace environment {
namespace mstc {

PartiallyOccupiableIdentifiableCell::PartiallyOccupiableIdentifiableCell(
    PointPtr center, double size, std::string robot_name) :
    IdentifiableCell(center, size, robot_name) {
}

PartiallyOccupiableIdentifiableCell::~PartiallyOccupiableIdentifiableCell() {
}

PointPtr PartiallyOccupiableIdentifiableCell::_center() {
  return get_center();
}

double PartiallyOccupiableIdentifiableCell::_size() {
  return get_size();
}

}
}
}

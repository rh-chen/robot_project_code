// Generated by gencpp from file potential_exploration/WaypointPropositionRequest.msg
// DO NOT EDIT!


#ifndef POTENTIAL_EXPLORATION_MESSAGE_WAYPOINTPROPOSITIONREQUEST_H
#define POTENTIAL_EXPLORATION_MESSAGE_WAYPOINTPROPOSITIONREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <geometry_msgs/Pose2D.h>

namespace potential_exploration
{
template <class ContainerAllocator>
struct WaypointPropositionRequest_
{
  typedef WaypointPropositionRequest_<ContainerAllocator> Type;

  WaypointPropositionRequest_()
    : waypoint()  {
    }
  WaypointPropositionRequest_(const ContainerAllocator& _alloc)
    : waypoint(_alloc)  {
  (void)_alloc;
    }



   typedef  ::geometry_msgs::Pose2D_<ContainerAllocator>  _waypoint_type;
  _waypoint_type waypoint;





  typedef boost::shared_ptr< ::potential_exploration::WaypointPropositionRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::potential_exploration::WaypointPropositionRequest_<ContainerAllocator> const> ConstPtr;

}; // struct WaypointPropositionRequest_

typedef ::potential_exploration::WaypointPropositionRequest_<std::allocator<void> > WaypointPropositionRequest;

typedef boost::shared_ptr< ::potential_exploration::WaypointPropositionRequest > WaypointPropositionRequestPtr;
typedef boost::shared_ptr< ::potential_exploration::WaypointPropositionRequest const> WaypointPropositionRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::potential_exploration::WaypointPropositionRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::potential_exploration::WaypointPropositionRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace potential_exploration

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'nav_msgs': ['/opt/ros/kinetic/share/nav_msgs/cmake/../msg'], 'potential_exploration': ['/home/firefly/potential_exploration_ros/src/potential_exploration/msg'], 'actionlib_msgs': ['/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::potential_exploration::WaypointPropositionRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::potential_exploration::WaypointPropositionRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::potential_exploration::WaypointPropositionRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::potential_exploration::WaypointPropositionRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::potential_exploration::WaypointPropositionRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::potential_exploration::WaypointPropositionRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::potential_exploration::WaypointPropositionRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "a56beb2234569122887d457fa6b14208";
  }

  static const char* value(const ::potential_exploration::WaypointPropositionRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xa56beb2234569122ULL;
  static const uint64_t static_value2 = 0x887d457fa6b14208ULL;
};

template<class ContainerAllocator>
struct DataType< ::potential_exploration::WaypointPropositionRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "potential_exploration/WaypointPropositionRequest";
  }

  static const char* value(const ::potential_exploration::WaypointPropositionRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::potential_exploration::WaypointPropositionRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n\
geometry_msgs/Pose2D waypoint\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Pose2D\n\
# This expresses a position and orientation on a 2D manifold.\n\
\n\
float64 x\n\
float64 y\n\
float64 theta\n\
";
  }

  static const char* value(const ::potential_exploration::WaypointPropositionRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::potential_exploration::WaypointPropositionRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.waypoint);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct WaypointPropositionRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::potential_exploration::WaypointPropositionRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::potential_exploration::WaypointPropositionRequest_<ContainerAllocator>& v)
  {
    s << indent << "waypoint: ";
    s << std::endl;
    Printer< ::geometry_msgs::Pose2D_<ContainerAllocator> >::stream(s, indent + "  ", v.waypoint);
  }
};

} // namespace message_operations
} // namespace ros

#endif // POTENTIAL_EXPLORATION_MESSAGE_WAYPOINTPROPOSITIONREQUEST_H

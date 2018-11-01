/* Software License Agreement (BSD License)
 *
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of Willow Garage, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Auto-generated by genmsg_cpp from file /home/josh/xme/examples/ROSGateway/src/application/ros_system/srv/SubstractTwoInts.srv
 *
 */


#ifndef ROS_SYSTEM_MESSAGE_SUBSTRACTTWOINTSRESPONSE_H
#define ROS_SYSTEM_MESSAGE_SUBSTRACTTWOINTSRESPONSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace ros_system
{
template <class ContainerAllocator>
struct SubstractTwoIntsResponse_
{
  typedef SubstractTwoIntsResponse_<ContainerAllocator> Type;

  SubstractTwoIntsResponse_()
    : difference(0)  {
    }
  SubstractTwoIntsResponse_(const ContainerAllocator& _alloc)
    : difference(0)  {
    }



   typedef int64_t _difference_type;
  _difference_type difference;




  typedef boost::shared_ptr< ::ros_system::SubstractTwoIntsResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::ros_system::SubstractTwoIntsResponse_<ContainerAllocator> const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;

}; // struct SubstractTwoIntsResponse_

typedef ::ros_system::SubstractTwoIntsResponse_<std::allocator<void> > SubstractTwoIntsResponse;

typedef boost::shared_ptr< ::ros_system::SubstractTwoIntsResponse > SubstractTwoIntsResponsePtr;
typedef boost::shared_ptr< ::ros_system::SubstractTwoIntsResponse const> SubstractTwoIntsResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::ros_system::SubstractTwoIntsResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::ros_system::SubstractTwoIntsResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace ros_system

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/groovy/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::ros_system::SubstractTwoIntsResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ros_system::SubstractTwoIntsResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ros_system::SubstractTwoIntsResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ros_system::SubstractTwoIntsResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ros_system::SubstractTwoIntsResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ros_system::SubstractTwoIntsResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::ros_system::SubstractTwoIntsResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "cadc8aeedf904e978a78ed9f0fc8b2a7";
  }

  static const char* value(const ::ros_system::SubstractTwoIntsResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xcadc8aeedf904e97ULL;
  static const uint64_t static_value2 = 0x8a78ed9f0fc8b2a7ULL;
};

template<class ContainerAllocator>
struct DataType< ::ros_system::SubstractTwoIntsResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ros_system/SubstractTwoIntsResponse";
  }

  static const char* value(const ::ros_system::SubstractTwoIntsResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::ros_system::SubstractTwoIntsResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int64 difference\n\
\n\
\n\
";
  }

  static const char* value(const ::ros_system::SubstractTwoIntsResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::ros_system::SubstractTwoIntsResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.difference);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER;
  }; // struct SubstractTwoIntsResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::ros_system::SubstractTwoIntsResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::ros_system::SubstractTwoIntsResponse_<ContainerAllocator>& v)
  {
    s << indent << "difference: ";
    Printer<int64_t>::stream(s, indent + "  ", v.difference);
  }
};

} // namespace message_operations
} // namespace ros

#endif // ROS_SYSTEM_MESSAGE_SUBSTRACTTWOINTSRESPONSE_H

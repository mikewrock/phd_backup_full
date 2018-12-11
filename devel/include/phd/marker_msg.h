// Generated by gencpp from file phd/marker_msg.msg
// DO NOT EDIT!


#ifndef PHD_MESSAGE_MARKER_MSG_H
#define PHD_MESSAGE_MARKER_MSG_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point.h>

namespace phd
{
template <class ContainerAllocator>
struct marker_msg_
{
  typedef marker_msg_<ContainerAllocator> Type;

  marker_msg_()
    : p1()
    , p2()
    , p3()
    , vec1()
    , vec2()
    , transform()
    , VAL1(0.0)
    , VAL2(0.0)
    , VAL3(0.0)
    , VAL4(0.0)  {
    }
  marker_msg_(const ContainerAllocator& _alloc)
    : p1(_alloc)
    , p2(_alloc)
    , p3(_alloc)
    , vec1(_alloc)
    , vec2(_alloc)
    , transform(_alloc)
    , VAL1(0.0)
    , VAL2(0.0)
    , VAL3(0.0)
    , VAL4(0.0)  {
  (void)_alloc;
    }



   typedef  ::geometry_msgs::Point_<ContainerAllocator>  _p1_type;
  _p1_type p1;

   typedef  ::geometry_msgs::Point_<ContainerAllocator>  _p2_type;
  _p2_type p2;

   typedef  ::geometry_msgs::Point_<ContainerAllocator>  _p3_type;
  _p3_type p3;

   typedef  ::geometry_msgs::Point_<ContainerAllocator>  _vec1_type;
  _vec1_type vec1;

   typedef  ::geometry_msgs::Point_<ContainerAllocator>  _vec2_type;
  _vec2_type vec2;

   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _transform_type;
  _transform_type transform;

   typedef float _VAL1_type;
  _VAL1_type VAL1;

   typedef float _VAL2_type;
  _VAL2_type VAL2;

   typedef float _VAL3_type;
  _VAL3_type VAL3;

   typedef float _VAL4_type;
  _VAL4_type VAL4;




  typedef boost::shared_ptr< ::phd::marker_msg_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::phd::marker_msg_<ContainerAllocator> const> ConstPtr;

}; // struct marker_msg_

typedef ::phd::marker_msg_<std::allocator<void> > marker_msg;

typedef boost::shared_ptr< ::phd::marker_msg > marker_msgPtr;
typedef boost::shared_ptr< ::phd::marker_msg const> marker_msgConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::phd::marker_msg_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::phd::marker_msg_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace phd

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'sensor_msgs': ['/opt/ros/indigo/share/sensor_msgs/cmake/../msg'], 'actionlib_msgs': ['/opt/ros/indigo/share/actionlib_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/indigo/share/std_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/indigo/share/geometry_msgs/cmake/../msg'], 'phd': ['/home/mike/catkin_ws/src/phd/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::phd::marker_msg_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::phd::marker_msg_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::phd::marker_msg_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::phd::marker_msg_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::phd::marker_msg_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::phd::marker_msg_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::phd::marker_msg_<ContainerAllocator> >
{
  static const char* value()
  {
    return "46163b1a11ff6053bb2e316cbd0d9ada";
  }

  static const char* value(const ::phd::marker_msg_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x46163b1a11ff6053ULL;
  static const uint64_t static_value2 = 0xbb2e316cbd0d9adaULL;
};

template<class ContainerAllocator>
struct DataType< ::phd::marker_msg_<ContainerAllocator> >
{
  static const char* value()
  {
    return "phd/marker_msg";
  }

  static const char* value(const ::phd::marker_msg_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::phd::marker_msg_<ContainerAllocator> >
{
  static const char* value()
  {
    return "geometry_msgs/Point p1\n\
geometry_msgs/Point p2\n\
geometry_msgs/Point p3\n\
geometry_msgs/Point vec1\n\
geometry_msgs/Point vec2\n\
float64[] transform\n\
float32 VAL1\n\
float32 VAL2\n\
float32 VAL3\n\
float32 VAL4\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Point\n\
# This contains the position of a point in free space\n\
float64 x\n\
float64 y\n\
float64 z\n\
";
  }

  static const char* value(const ::phd::marker_msg_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::phd::marker_msg_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.p1);
      stream.next(m.p2);
      stream.next(m.p3);
      stream.next(m.vec1);
      stream.next(m.vec2);
      stream.next(m.transform);
      stream.next(m.VAL1);
      stream.next(m.VAL2);
      stream.next(m.VAL3);
      stream.next(m.VAL4);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct marker_msg_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::phd::marker_msg_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::phd::marker_msg_<ContainerAllocator>& v)
  {
    s << indent << "p1: ";
    s << std::endl;
    Printer< ::geometry_msgs::Point_<ContainerAllocator> >::stream(s, indent + "  ", v.p1);
    s << indent << "p2: ";
    s << std::endl;
    Printer< ::geometry_msgs::Point_<ContainerAllocator> >::stream(s, indent + "  ", v.p2);
    s << indent << "p3: ";
    s << std::endl;
    Printer< ::geometry_msgs::Point_<ContainerAllocator> >::stream(s, indent + "  ", v.p3);
    s << indent << "vec1: ";
    s << std::endl;
    Printer< ::geometry_msgs::Point_<ContainerAllocator> >::stream(s, indent + "  ", v.vec1);
    s << indent << "vec2: ";
    s << std::endl;
    Printer< ::geometry_msgs::Point_<ContainerAllocator> >::stream(s, indent + "  ", v.vec2);
    s << indent << "transform[]" << std::endl;
    for (size_t i = 0; i < v.transform.size(); ++i)
    {
      s << indent << "  transform[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.transform[i]);
    }
    s << indent << "VAL1: ";
    Printer<float>::stream(s, indent + "  ", v.VAL1);
    s << indent << "VAL2: ";
    Printer<float>::stream(s, indent + "  ", v.VAL2);
    s << indent << "VAL3: ";
    Printer<float>::stream(s, indent + "  ", v.VAL3);
    s << indent << "VAL4: ";
    Printer<float>::stream(s, indent + "  ", v.VAL4);
  }
};

} // namespace message_operations
} // namespace ros

#endif // PHD_MESSAGE_MARKER_MSG_H

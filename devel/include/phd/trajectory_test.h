// Generated by gencpp from file phd/trajectory_test.msg
// DO NOT EDIT!


#ifndef PHD_MESSAGE_TRAJECTORY_TEST_H
#define PHD_MESSAGE_TRAJECTORY_TEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace phd
{
template <class ContainerAllocator>
struct trajectory_test_
{
  typedef trajectory_test_<ContainerAllocator> Type;

  trajectory_test_()
    : x(0.0)  {
    }
  trajectory_test_(const ContainerAllocator& _alloc)
    : x(0.0)  {
  (void)_alloc;
    }



   typedef float _x_type;
  _x_type x;




  typedef boost::shared_ptr< ::phd::trajectory_test_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::phd::trajectory_test_<ContainerAllocator> const> ConstPtr;

}; // struct trajectory_test_

typedef ::phd::trajectory_test_<std::allocator<void> > trajectory_test;

typedef boost::shared_ptr< ::phd::trajectory_test > trajectory_testPtr;
typedef boost::shared_ptr< ::phd::trajectory_test const> trajectory_testConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::phd::trajectory_test_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::phd::trajectory_test_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace phd

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'sensor_msgs': ['/opt/ros/indigo/share/sensor_msgs/cmake/../msg'], 'actionlib_msgs': ['/opt/ros/indigo/share/actionlib_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/indigo/share/std_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/indigo/share/geometry_msgs/cmake/../msg'], 'phd': ['/home/mike/catkin_ws/src/phd/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::phd::trajectory_test_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::phd::trajectory_test_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::phd::trajectory_test_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::phd::trajectory_test_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::phd::trajectory_test_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::phd::trajectory_test_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::phd::trajectory_test_<ContainerAllocator> >
{
  static const char* value()
  {
    return "abd5d1e9c3ac157a0df3ba27b65d3384";
  }

  static const char* value(const ::phd::trajectory_test_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xabd5d1e9c3ac157aULL;
  static const uint64_t static_value2 = 0x0df3ba27b65d3384ULL;
};

template<class ContainerAllocator>
struct DataType< ::phd::trajectory_test_<ContainerAllocator> >
{
  static const char* value()
  {
    return "phd/trajectory_test";
  }

  static const char* value(const ::phd::trajectory_test_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::phd::trajectory_test_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float32 x\n\
\n\
";
  }

  static const char* value(const ::phd::trajectory_test_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::phd::trajectory_test_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.x);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct trajectory_test_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::phd::trajectory_test_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::phd::trajectory_test_<ContainerAllocator>& v)
  {
    s << indent << "x: ";
    Printer<float>::stream(s, indent + "  ", v.x);
  }
};

} // namespace message_operations
} // namespace ros

#endif // PHD_MESSAGE_TRAJECTORY_TEST_H

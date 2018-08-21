// Generated by gencpp from file phd/trajectory_point.msg
// DO NOT EDIT!


#ifndef PHD_MESSAGE_TRAJECTORY_POINT_H
#define PHD_MESSAGE_TRAJECTORY_POINT_H


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
struct trajectory_point_
{
  typedef trajectory_point_<ContainerAllocator> Type;

  trajectory_point_()
    : x(0.0)
    , y(0.0)
    , z(0.0)
    , nx(0.0)
    , ny(0.0)
    , nz(0.0)
    , d(0.0)  {
    }
  trajectory_point_(const ContainerAllocator& _alloc)
    : x(0.0)
    , y(0.0)
    , z(0.0)
    , nx(0.0)
    , ny(0.0)
    , nz(0.0)
    , d(0.0)  {
  (void)_alloc;
    }



   typedef float _x_type;
  _x_type x;

   typedef float _y_type;
  _y_type y;

   typedef float _z_type;
  _z_type z;

   typedef float _nx_type;
  _nx_type nx;

   typedef float _ny_type;
  _ny_type ny;

   typedef float _nz_type;
  _nz_type nz;

   typedef float _d_type;
  _d_type d;




  typedef boost::shared_ptr< ::phd::trajectory_point_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::phd::trajectory_point_<ContainerAllocator> const> ConstPtr;

}; // struct trajectory_point_

typedef ::phd::trajectory_point_<std::allocator<void> > trajectory_point;

typedef boost::shared_ptr< ::phd::trajectory_point > trajectory_pointPtr;
typedef boost::shared_ptr< ::phd::trajectory_point const> trajectory_pointConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::phd::trajectory_point_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::phd::trajectory_point_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::phd::trajectory_point_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::phd::trajectory_point_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::phd::trajectory_point_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::phd::trajectory_point_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::phd::trajectory_point_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::phd::trajectory_point_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::phd::trajectory_point_<ContainerAllocator> >
{
  static const char* value()
  {
    return "a8f5392abe73cf46e23084bf4c802525";
  }

  static const char* value(const ::phd::trajectory_point_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xa8f5392abe73cf46ULL;
  static const uint64_t static_value2 = 0xe23084bf4c802525ULL;
};

template<class ContainerAllocator>
struct DataType< ::phd::trajectory_point_<ContainerAllocator> >
{
  static const char* value()
  {
    return "phd/trajectory_point";
  }

  static const char* value(const ::phd::trajectory_point_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::phd::trajectory_point_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float32 x\n\
float32 y\n\
float32 z\n\
float32 nx\n\
float32 ny\n\
float32 nz\n\
float32 d\n\
";
  }

  static const char* value(const ::phd::trajectory_point_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::phd::trajectory_point_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.x);
      stream.next(m.y);
      stream.next(m.z);
      stream.next(m.nx);
      stream.next(m.ny);
      stream.next(m.nz);
      stream.next(m.d);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct trajectory_point_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::phd::trajectory_point_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::phd::trajectory_point_<ContainerAllocator>& v)
  {
    s << indent << "x: ";
    Printer<float>::stream(s, indent + "  ", v.x);
    s << indent << "y: ";
    Printer<float>::stream(s, indent + "  ", v.y);
    s << indent << "z: ";
    Printer<float>::stream(s, indent + "  ", v.z);
    s << indent << "nx: ";
    Printer<float>::stream(s, indent + "  ", v.nx);
    s << indent << "ny: ";
    Printer<float>::stream(s, indent + "  ", v.ny);
    s << indent << "nz: ";
    Printer<float>::stream(s, indent + "  ", v.nz);
    s << indent << "d: ";
    Printer<float>::stream(s, indent + "  ", v.d);
  }
};

} // namespace message_operations
} // namespace ros

#endif // PHD_MESSAGE_TRAJECTORY_POINT_H

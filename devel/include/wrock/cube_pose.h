// Generated by gencpp from file wrock/cube_pose.msg
// DO NOT EDIT!


#ifndef WROCK_MESSAGE_CUBE_POSE_H
#define WROCK_MESSAGE_CUBE_POSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace wrock
{
template <class ContainerAllocator>
struct cube_pose_
{
  typedef cube_pose_<ContainerAllocator> Type;

  cube_pose_()
    : x(0.0)
    , y(0.0)
    , z(0.0)
    , pitch(0.0)
    , yaw(0.0)
    , xrot(0.0)
    , yrot(0.0)
    , zrot(0.0)  {
    }
  cube_pose_(const ContainerAllocator& _alloc)
    : x(0.0)
    , y(0.0)
    , z(0.0)
    , pitch(0.0)
    , yaw(0.0)
    , xrot(0.0)
    , yrot(0.0)
    , zrot(0.0)  {
  (void)_alloc;
    }



   typedef float _x_type;
  _x_type x;

   typedef float _y_type;
  _y_type y;

   typedef float _z_type;
  _z_type z;

   typedef float _pitch_type;
  _pitch_type pitch;

   typedef float _yaw_type;
  _yaw_type yaw;

   typedef float _xrot_type;
  _xrot_type xrot;

   typedef float _yrot_type;
  _yrot_type yrot;

   typedef float _zrot_type;
  _zrot_type zrot;




  typedef boost::shared_ptr< ::wrock::cube_pose_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::wrock::cube_pose_<ContainerAllocator> const> ConstPtr;

}; // struct cube_pose_

typedef ::wrock::cube_pose_<std::allocator<void> > cube_pose;

typedef boost::shared_ptr< ::wrock::cube_pose > cube_posePtr;
typedef boost::shared_ptr< ::wrock::cube_pose const> cube_poseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::wrock::cube_pose_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::wrock::cube_pose_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace wrock

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'sensor_msgs': ['/opt/ros/indigo/share/sensor_msgs/cmake/../msg'], 'wrock': ['/home/mike/catkin_ws/src/wrock/msg'], 'actionlib_msgs': ['/opt/ros/indigo/share/actionlib_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/indigo/share/std_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/indigo/share/geometry_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::wrock::cube_pose_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::wrock::cube_pose_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::wrock::cube_pose_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::wrock::cube_pose_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::wrock::cube_pose_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::wrock::cube_pose_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::wrock::cube_pose_<ContainerAllocator> >
{
  static const char* value()
  {
    return "9b97735605d3278f4692f5c037707560";
  }

  static const char* value(const ::wrock::cube_pose_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x9b97735605d3278fULL;
  static const uint64_t static_value2 = 0x4692f5c037707560ULL;
};

template<class ContainerAllocator>
struct DataType< ::wrock::cube_pose_<ContainerAllocator> >
{
  static const char* value()
  {
    return "wrock/cube_pose";
  }

  static const char* value(const ::wrock::cube_pose_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::wrock::cube_pose_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float32 x\n\
float32 y\n\
float32 z\n\
float32 pitch\n\
float32 yaw\n\
float32 xrot\n\
float32 yrot\n\
float32 zrot\n\
\n\
";
  }

  static const char* value(const ::wrock::cube_pose_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::wrock::cube_pose_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.x);
      stream.next(m.y);
      stream.next(m.z);
      stream.next(m.pitch);
      stream.next(m.yaw);
      stream.next(m.xrot);
      stream.next(m.yrot);
      stream.next(m.zrot);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct cube_pose_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::wrock::cube_pose_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::wrock::cube_pose_<ContainerAllocator>& v)
  {
    s << indent << "x: ";
    Printer<float>::stream(s, indent + "  ", v.x);
    s << indent << "y: ";
    Printer<float>::stream(s, indent + "  ", v.y);
    s << indent << "z: ";
    Printer<float>::stream(s, indent + "  ", v.z);
    s << indent << "pitch: ";
    Printer<float>::stream(s, indent + "  ", v.pitch);
    s << indent << "yaw: ";
    Printer<float>::stream(s, indent + "  ", v.yaw);
    s << indent << "xrot: ";
    Printer<float>::stream(s, indent + "  ", v.xrot);
    s << indent << "yrot: ";
    Printer<float>::stream(s, indent + "  ", v.yrot);
    s << indent << "zrot: ";
    Printer<float>::stream(s, indent + "  ", v.zrot);
  }
};

} // namespace message_operations
} // namespace ros

#endif // WROCK_MESSAGE_CUBE_POSE_H

// Generated by gencpp from file phd/doctor_cloudResponse.msg
// DO NOT EDIT!


#ifndef PHD_MESSAGE_DOCTOR_CLOUDRESPONSE_H
#define PHD_MESSAGE_DOCTOR_CLOUDRESPONSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <phd/doctor_msg.h>

namespace phd
{
template <class ContainerAllocator>
struct doctor_cloudResponse_
{
  typedef doctor_cloudResponse_<ContainerAllocator> Type;

  doctor_cloudResponse_()
    : clouds()  {
    }
  doctor_cloudResponse_(const ContainerAllocator& _alloc)
    : clouds(_alloc)  {
  (void)_alloc;
    }



   typedef std::vector< ::phd::doctor_msg_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::phd::doctor_msg_<ContainerAllocator> >::other >  _clouds_type;
  _clouds_type clouds;




  typedef boost::shared_ptr< ::phd::doctor_cloudResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::phd::doctor_cloudResponse_<ContainerAllocator> const> ConstPtr;

}; // struct doctor_cloudResponse_

typedef ::phd::doctor_cloudResponse_<std::allocator<void> > doctor_cloudResponse;

typedef boost::shared_ptr< ::phd::doctor_cloudResponse > doctor_cloudResponsePtr;
typedef boost::shared_ptr< ::phd::doctor_cloudResponse const> doctor_cloudResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::phd::doctor_cloudResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::phd::doctor_cloudResponse_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::phd::doctor_cloudResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::phd::doctor_cloudResponse_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::phd::doctor_cloudResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::phd::doctor_cloudResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::phd::doctor_cloudResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::phd::doctor_cloudResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::phd::doctor_cloudResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "2f19691a84376a2883697fa8233da957";
  }

  static const char* value(const ::phd::doctor_cloudResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x2f19691a84376a28ULL;
  static const uint64_t static_value2 = 0x83697fa8233da957ULL;
};

template<class ContainerAllocator>
struct DataType< ::phd::doctor_cloudResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "phd/doctor_cloudResponse";
  }

  static const char* value(const ::phd::doctor_cloudResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::phd::doctor_cloudResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "doctor_msg[] clouds\n\
\n\
\n\
================================================================================\n\
MSG: phd/doctor_msg\n\
sensor_msgs/PointCloud2 cloud_out\n\
float64[16] transform_mat\n\
float32 x\n\
float32 y\n\
float32 z\n\
float32 id\n\
\n\
================================================================================\n\
MSG: sensor_msgs/PointCloud2\n\
# This message holds a collection of N-dimensional points, which may\n\
# contain additional information such as normals, intensity, etc. The\n\
# point data is stored as a binary blob, its layout described by the\n\
# contents of the \"fields\" array.\n\
\n\
# The point cloud data may be organized 2d (image-like) or 1d\n\
# (unordered). Point clouds organized as 2d images may be produced by\n\
# camera depth sensors such as stereo or time-of-flight.\n\
\n\
# Time of sensor data acquisition, and the coordinate frame ID (for 3d\n\
# points).\n\
Header header\n\
\n\
# 2D structure of the point cloud. If the cloud is unordered, height is\n\
# 1 and width is the length of the point cloud.\n\
uint32 height\n\
uint32 width\n\
\n\
# Describes the channels and their layout in the binary data blob.\n\
PointField[] fields\n\
\n\
bool    is_bigendian # Is this data bigendian?\n\
uint32  point_step   # Length of a point in bytes\n\
uint32  row_step     # Length of a row in bytes\n\
uint8[] data         # Actual point data, size is (row_step*height)\n\
\n\
bool is_dense        # True if there are no invalid points\n\
\n\
================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n\
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
\n\
================================================================================\n\
MSG: sensor_msgs/PointField\n\
# This message holds the description of one point entry in the\n\
# PointCloud2 message format.\n\
uint8 INT8    = 1\n\
uint8 UINT8   = 2\n\
uint8 INT16   = 3\n\
uint8 UINT16  = 4\n\
uint8 INT32   = 5\n\
uint8 UINT32  = 6\n\
uint8 FLOAT32 = 7\n\
uint8 FLOAT64 = 8\n\
\n\
string name      # Name of field\n\
uint32 offset    # Offset from start of point struct\n\
uint8  datatype  # Datatype enumeration, see above\n\
uint32 count     # How many elements in the field\n\
";
  }

  static const char* value(const ::phd::doctor_cloudResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::phd::doctor_cloudResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.clouds);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct doctor_cloudResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::phd::doctor_cloudResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::phd::doctor_cloudResponse_<ContainerAllocator>& v)
  {
    s << indent << "clouds[]" << std::endl;
    for (size_t i = 0; i < v.clouds.size(); ++i)
    {
      s << indent << "  clouds[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::phd::doctor_msg_<ContainerAllocator> >::stream(s, indent + "    ", v.clouds[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // PHD_MESSAGE_DOCTOR_CLOUDRESPONSE_H
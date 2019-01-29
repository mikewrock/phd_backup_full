// Generated by gencpp from file phd/calc_serviceRequest.msg
// DO NOT EDIT!


#ifndef PHD_MESSAGE_CALC_SERVICEREQUEST_H
#define PHD_MESSAGE_CALC_SERVICEREQUEST_H


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
struct calc_serviceRequest_
{
  typedef calc_serviceRequest_<ContainerAllocator> Type;

  calc_serviceRequest_()
    : pre_ids()
    , post_ids()
    , datum()
    , location()  {
    }
  calc_serviceRequest_(const ContainerAllocator& _alloc)
    : pre_ids(_alloc)
    , post_ids(_alloc)
    , datum(_alloc)
    , location(_alloc)  {
  (void)_alloc;
    }



   typedef std::vector<int32_t, typename ContainerAllocator::template rebind<int32_t>::other >  _pre_ids_type;
  _pre_ids_type pre_ids;

   typedef std::vector<int32_t, typename ContainerAllocator::template rebind<int32_t>::other >  _post_ids_type;
  _post_ids_type post_ids;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _datum_type;
  _datum_type datum;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _location_type;
  _location_type location;




  typedef boost::shared_ptr< ::phd::calc_serviceRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::phd::calc_serviceRequest_<ContainerAllocator> const> ConstPtr;

}; // struct calc_serviceRequest_

typedef ::phd::calc_serviceRequest_<std::allocator<void> > calc_serviceRequest;

typedef boost::shared_ptr< ::phd::calc_serviceRequest > calc_serviceRequestPtr;
typedef boost::shared_ptr< ::phd::calc_serviceRequest const> calc_serviceRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::phd::calc_serviceRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::phd::calc_serviceRequest_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::phd::calc_serviceRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::phd::calc_serviceRequest_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::phd::calc_serviceRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::phd::calc_serviceRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::phd::calc_serviceRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::phd::calc_serviceRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::phd::calc_serviceRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "504533770c671b8893346f8f23298fee";
  }

  static const char* value(const ::phd::calc_serviceRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x504533770c671b88ULL;
  static const uint64_t static_value2 = 0x93346f8f23298feeULL;
};

template<class ContainerAllocator>
struct DataType< ::phd::calc_serviceRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "phd/calc_serviceRequest";
  }

  static const char* value(const ::phd::calc_serviceRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::phd::calc_serviceRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int32[] pre_ids\n\
int32[] post_ids\n\
string datum\n\
string location\n\
";
  }

  static const char* value(const ::phd::calc_serviceRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::phd::calc_serviceRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.pre_ids);
      stream.next(m.post_ids);
      stream.next(m.datum);
      stream.next(m.location);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct calc_serviceRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::phd::calc_serviceRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::phd::calc_serviceRequest_<ContainerAllocator>& v)
  {
    s << indent << "pre_ids[]" << std::endl;
    for (size_t i = 0; i < v.pre_ids.size(); ++i)
    {
      s << indent << "  pre_ids[" << i << "]: ";
      Printer<int32_t>::stream(s, indent + "  ", v.pre_ids[i]);
    }
    s << indent << "post_ids[]" << std::endl;
    for (size_t i = 0; i < v.post_ids.size(); ++i)
    {
      s << indent << "  post_ids[" << i << "]: ";
      Printer<int32_t>::stream(s, indent + "  ", v.post_ids[i]);
    }
    s << indent << "datum: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.datum);
    s << indent << "location: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.location);
  }
};

} // namespace message_operations
} // namespace ros

#endif // PHD_MESSAGE_CALC_SERVICEREQUEST_H
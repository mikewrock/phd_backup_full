// Generated by gencpp from file phd/trajectory_service.msg
// DO NOT EDIT!


#ifndef PHD_MESSAGE_TRAJECTORY_SERVICE_H
#define PHD_MESSAGE_TRAJECTORY_SERVICE_H

#include <ros/service_traits.h>


#include <phd/trajectory_serviceRequest.h>
#include <phd/trajectory_serviceResponse.h>


namespace phd
{

struct trajectory_service
{

typedef trajectory_serviceRequest Request;
typedef trajectory_serviceResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct trajectory_service
} // namespace phd


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::phd::trajectory_service > {
  static const char* value()
  {
    return "406cb769178a6d411779375e331fbd67";
  }

  static const char* value(const ::phd::trajectory_service&) { return value(); }
};

template<>
struct DataType< ::phd::trajectory_service > {
  static const char* value()
  {
    return "phd/trajectory_service";
  }

  static const char* value(const ::phd::trajectory_service&) { return value(); }
};


// service_traits::MD5Sum< ::phd::trajectory_serviceRequest> should match 
// service_traits::MD5Sum< ::phd::trajectory_service > 
template<>
struct MD5Sum< ::phd::trajectory_serviceRequest>
{
  static const char* value()
  {
    return MD5Sum< ::phd::trajectory_service >::value();
  }
  static const char* value(const ::phd::trajectory_serviceRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::phd::trajectory_serviceRequest> should match 
// service_traits::DataType< ::phd::trajectory_service > 
template<>
struct DataType< ::phd::trajectory_serviceRequest>
{
  static const char* value()
  {
    return DataType< ::phd::trajectory_service >::value();
  }
  static const char* value(const ::phd::trajectory_serviceRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::phd::trajectory_serviceResponse> should match 
// service_traits::MD5Sum< ::phd::trajectory_service > 
template<>
struct MD5Sum< ::phd::trajectory_serviceResponse>
{
  static const char* value()
  {
    return MD5Sum< ::phd::trajectory_service >::value();
  }
  static const char* value(const ::phd::trajectory_serviceResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::phd::trajectory_serviceResponse> should match 
// service_traits::DataType< ::phd::trajectory_service > 
template<>
struct DataType< ::phd::trajectory_serviceResponse>
{
  static const char* value()
  {
    return DataType< ::phd::trajectory_service >::value();
  }
  static const char* value(const ::phd::trajectory_serviceResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // PHD_MESSAGE_TRAJECTORY_SERVICE_H

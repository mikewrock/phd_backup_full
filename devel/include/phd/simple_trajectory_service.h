// Generated by gencpp from file phd/simple_trajectory_service.msg
// DO NOT EDIT!


#ifndef PHD_MESSAGE_SIMPLE_TRAJECTORY_SERVICE_H
#define PHD_MESSAGE_SIMPLE_TRAJECTORY_SERVICE_H

#include <ros/service_traits.h>


#include <phd/simple_trajectory_serviceRequest.h>
#include <phd/simple_trajectory_serviceResponse.h>


namespace phd
{

struct simple_trajectory_service
{

typedef simple_trajectory_serviceRequest Request;
typedef simple_trajectory_serviceResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct simple_trajectory_service
} // namespace phd


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::phd::simple_trajectory_service > {
  static const char* value()
  {
    return "b21993cb52c2540858ed98b988a9faf4";
  }

  static const char* value(const ::phd::simple_trajectory_service&) { return value(); }
};

template<>
struct DataType< ::phd::simple_trajectory_service > {
  static const char* value()
  {
    return "phd/simple_trajectory_service";
  }

  static const char* value(const ::phd::simple_trajectory_service&) { return value(); }
};


// service_traits::MD5Sum< ::phd::simple_trajectory_serviceRequest> should match 
// service_traits::MD5Sum< ::phd::simple_trajectory_service > 
template<>
struct MD5Sum< ::phd::simple_trajectory_serviceRequest>
{
  static const char* value()
  {
    return MD5Sum< ::phd::simple_trajectory_service >::value();
  }
  static const char* value(const ::phd::simple_trajectory_serviceRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::phd::simple_trajectory_serviceRequest> should match 
// service_traits::DataType< ::phd::simple_trajectory_service > 
template<>
struct DataType< ::phd::simple_trajectory_serviceRequest>
{
  static const char* value()
  {
    return DataType< ::phd::simple_trajectory_service >::value();
  }
  static const char* value(const ::phd::simple_trajectory_serviceRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::phd::simple_trajectory_serviceResponse> should match 
// service_traits::MD5Sum< ::phd::simple_trajectory_service > 
template<>
struct MD5Sum< ::phd::simple_trajectory_serviceResponse>
{
  static const char* value()
  {
    return MD5Sum< ::phd::simple_trajectory_service >::value();
  }
  static const char* value(const ::phd::simple_trajectory_serviceResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::phd::simple_trajectory_serviceResponse> should match 
// service_traits::DataType< ::phd::simple_trajectory_service > 
template<>
struct DataType< ::phd::simple_trajectory_serviceResponse>
{
  static const char* value()
  {
    return DataType< ::phd::simple_trajectory_service >::value();
  }
  static const char* value(const ::phd::simple_trajectory_serviceResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // PHD_MESSAGE_SIMPLE_TRAJECTORY_SERVICE_H

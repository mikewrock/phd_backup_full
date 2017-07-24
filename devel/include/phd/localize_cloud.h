// Generated by gencpp from file phd/localize_cloud.msg
// DO NOT EDIT!


#ifndef PHD_MESSAGE_LOCALIZE_CLOUD_H
#define PHD_MESSAGE_LOCALIZE_CLOUD_H

#include <ros/service_traits.h>


#include <phd/localize_cloudRequest.h>
#include <phd/localize_cloudResponse.h>


namespace phd
{

struct localize_cloud
{

typedef localize_cloudRequest Request;
typedef localize_cloudResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct localize_cloud
} // namespace phd


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::phd::localize_cloud > {
  static const char* value()
  {
    return "b8092357b1c84167e3da82720a08fb1d";
  }

  static const char* value(const ::phd::localize_cloud&) { return value(); }
};

template<>
struct DataType< ::phd::localize_cloud > {
  static const char* value()
  {
    return "phd/localize_cloud";
  }

  static const char* value(const ::phd::localize_cloud&) { return value(); }
};


// service_traits::MD5Sum< ::phd::localize_cloudRequest> should match 
// service_traits::MD5Sum< ::phd::localize_cloud > 
template<>
struct MD5Sum< ::phd::localize_cloudRequest>
{
  static const char* value()
  {
    return MD5Sum< ::phd::localize_cloud >::value();
  }
  static const char* value(const ::phd::localize_cloudRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::phd::localize_cloudRequest> should match 
// service_traits::DataType< ::phd::localize_cloud > 
template<>
struct DataType< ::phd::localize_cloudRequest>
{
  static const char* value()
  {
    return DataType< ::phd::localize_cloud >::value();
  }
  static const char* value(const ::phd::localize_cloudRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::phd::localize_cloudResponse> should match 
// service_traits::MD5Sum< ::phd::localize_cloud > 
template<>
struct MD5Sum< ::phd::localize_cloudResponse>
{
  static const char* value()
  {
    return MD5Sum< ::phd::localize_cloud >::value();
  }
  static const char* value(const ::phd::localize_cloudResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::phd::localize_cloudResponse> should match 
// service_traits::DataType< ::phd::localize_cloud > 
template<>
struct DataType< ::phd::localize_cloudResponse>
{
  static const char* value()
  {
    return DataType< ::phd::localize_cloud >::value();
  }
  static const char* value(const ::phd::localize_cloudResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // PHD_MESSAGE_LOCALIZE_CLOUD_H

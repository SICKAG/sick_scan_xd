// Generated by gencpp from file sick_scan/SickLocReflectorsForSupportActiveSrv.msg
// DO NOT EDIT!


#ifndef SICK_SCAN_MESSAGE_SICKLOCREFLECTORSFORSUPPORTACTIVESRV_H
#define SICK_SCAN_MESSAGE_SICKLOCREFLECTORSFORSUPPORTACTIVESRV_H

#include <ros/service_traits.h>


#include <sick_scan/SickLocReflectorsForSupportActiveSrvRequest.h>
#include <sick_scan/SickLocReflectorsForSupportActiveSrvResponse.h>


namespace sick_scan
{

struct SickLocReflectorsForSupportActiveSrv
{

typedef SickLocReflectorsForSupportActiveSrvRequest Request;
typedef SickLocReflectorsForSupportActiveSrvResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct SickLocReflectorsForSupportActiveSrv
} // namespace sick_scan


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::sick_scan::SickLocReflectorsForSupportActiveSrv > {
  static const char* value()
  {
    return "9bb3f90ef7a4dde50a9193067d4298d5";
  }

  static const char* value(const ::sick_scan::SickLocReflectorsForSupportActiveSrv&) { return value(); }
};

template<>
struct DataType< ::sick_scan::SickLocReflectorsForSupportActiveSrv > {
  static const char* value()
  {
    return "sick_scan/SickLocReflectorsForSupportActiveSrv";
  }

  static const char* value(const ::sick_scan::SickLocReflectorsForSupportActiveSrv&) { return value(); }
};


// service_traits::MD5Sum< ::sick_scan::SickLocReflectorsForSupportActiveSrvRequest> should match
// service_traits::MD5Sum< ::sick_scan::SickLocReflectorsForSupportActiveSrv >
template<>
struct MD5Sum< ::sick_scan::SickLocReflectorsForSupportActiveSrvRequest>
{
  static const char* value()
  {
    return MD5Sum< ::sick_scan::SickLocReflectorsForSupportActiveSrv >::value();
  }
  static const char* value(const ::sick_scan::SickLocReflectorsForSupportActiveSrvRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::sick_scan::SickLocReflectorsForSupportActiveSrvRequest> should match
// service_traits::DataType< ::sick_scan::SickLocReflectorsForSupportActiveSrv >
template<>
struct DataType< ::sick_scan::SickLocReflectorsForSupportActiveSrvRequest>
{
  static const char* value()
  {
    return DataType< ::sick_scan::SickLocReflectorsForSupportActiveSrv >::value();
  }
  static const char* value(const ::sick_scan::SickLocReflectorsForSupportActiveSrvRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::sick_scan::SickLocReflectorsForSupportActiveSrvResponse> should match
// service_traits::MD5Sum< ::sick_scan::SickLocReflectorsForSupportActiveSrv >
template<>
struct MD5Sum< ::sick_scan::SickLocReflectorsForSupportActiveSrvResponse>
{
  static const char* value()
  {
    return MD5Sum< ::sick_scan::SickLocReflectorsForSupportActiveSrv >::value();
  }
  static const char* value(const ::sick_scan::SickLocReflectorsForSupportActiveSrvResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::sick_scan::SickLocReflectorsForSupportActiveSrvResponse> should match
// service_traits::DataType< ::sick_scan::SickLocReflectorsForSupportActiveSrv >
template<>
struct DataType< ::sick_scan::SickLocReflectorsForSupportActiveSrvResponse>
{
  static const char* value()
  {
    return DataType< ::sick_scan::SickLocReflectorsForSupportActiveSrv >::value();
  }
  static const char* value(const ::sick_scan::SickLocReflectorsForSupportActiveSrvResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // SICK_SCAN_MESSAGE_SICKLOCREFLECTORSFORSUPPORTACTIVESRV_H
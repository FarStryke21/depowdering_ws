// Generated by gencpp from file realsense_cam/FindWorkspaceCenter.msg
// DO NOT EDIT!


#ifndef REALSENSE_CAM_MESSAGE_FINDWORKSPACECENTER_H
#define REALSENSE_CAM_MESSAGE_FINDWORKSPACECENTER_H

#include <ros/service_traits.h>


#include <realsense_cam/FindWorkspaceCenterRequest.h>
#include <realsense_cam/FindWorkspaceCenterResponse.h>


namespace realsense_cam
{

struct FindWorkspaceCenter
{

typedef FindWorkspaceCenterRequest Request;
typedef FindWorkspaceCenterResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct FindWorkspaceCenter
} // namespace realsense_cam


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::realsense_cam::FindWorkspaceCenter > {
  static const char* value()
  {
    return "bd4595b8ce50ecc3768e9c5dc7e441ab";
  }

  static const char* value(const ::realsense_cam::FindWorkspaceCenter&) { return value(); }
};

template<>
struct DataType< ::realsense_cam::FindWorkspaceCenter > {
  static const char* value()
  {
    return "realsense_cam/FindWorkspaceCenter";
  }

  static const char* value(const ::realsense_cam::FindWorkspaceCenter&) { return value(); }
};


// service_traits::MD5Sum< ::realsense_cam::FindWorkspaceCenterRequest> should match
// service_traits::MD5Sum< ::realsense_cam::FindWorkspaceCenter >
template<>
struct MD5Sum< ::realsense_cam::FindWorkspaceCenterRequest>
{
  static const char* value()
  {
    return MD5Sum< ::realsense_cam::FindWorkspaceCenter >::value();
  }
  static const char* value(const ::realsense_cam::FindWorkspaceCenterRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::realsense_cam::FindWorkspaceCenterRequest> should match
// service_traits::DataType< ::realsense_cam::FindWorkspaceCenter >
template<>
struct DataType< ::realsense_cam::FindWorkspaceCenterRequest>
{
  static const char* value()
  {
    return DataType< ::realsense_cam::FindWorkspaceCenter >::value();
  }
  static const char* value(const ::realsense_cam::FindWorkspaceCenterRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::realsense_cam::FindWorkspaceCenterResponse> should match
// service_traits::MD5Sum< ::realsense_cam::FindWorkspaceCenter >
template<>
struct MD5Sum< ::realsense_cam::FindWorkspaceCenterResponse>
{
  static const char* value()
  {
    return MD5Sum< ::realsense_cam::FindWorkspaceCenter >::value();
  }
  static const char* value(const ::realsense_cam::FindWorkspaceCenterResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::realsense_cam::FindWorkspaceCenterResponse> should match
// service_traits::DataType< ::realsense_cam::FindWorkspaceCenter >
template<>
struct DataType< ::realsense_cam::FindWorkspaceCenterResponse>
{
  static const char* value()
  {
    return DataType< ::realsense_cam::FindWorkspaceCenter >::value();
  }
  static const char* value(const ::realsense_cam::FindWorkspaceCenterResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // REALSENSE_CAM_MESSAGE_FINDWORKSPACECENTER_H

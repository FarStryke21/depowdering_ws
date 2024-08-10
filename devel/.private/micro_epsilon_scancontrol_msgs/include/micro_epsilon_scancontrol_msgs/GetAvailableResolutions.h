// Generated by gencpp from file micro_epsilon_scancontrol_msgs/GetAvailableResolutions.msg
// DO NOT EDIT!


#ifndef MICRO_EPSILON_SCANCONTROL_MSGS_MESSAGE_GETAVAILABLERESOLUTIONS_H
#define MICRO_EPSILON_SCANCONTROL_MSGS_MESSAGE_GETAVAILABLERESOLUTIONS_H

#include <ros/service_traits.h>


#include <micro_epsilon_scancontrol_msgs/GetAvailableResolutionsRequest.h>
#include <micro_epsilon_scancontrol_msgs/GetAvailableResolutionsResponse.h>


namespace micro_epsilon_scancontrol_msgs
{

struct GetAvailableResolutions
{

typedef GetAvailableResolutionsRequest Request;
typedef GetAvailableResolutionsResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct GetAvailableResolutions
} // namespace micro_epsilon_scancontrol_msgs


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::micro_epsilon_scancontrol_msgs::GetAvailableResolutions > {
  static const char* value()
  {
    return "a603ff8225a17994979733ebf35285f3";
  }

  static const char* value(const ::micro_epsilon_scancontrol_msgs::GetAvailableResolutions&) { return value(); }
};

template<>
struct DataType< ::micro_epsilon_scancontrol_msgs::GetAvailableResolutions > {
  static const char* value()
  {
    return "micro_epsilon_scancontrol_msgs/GetAvailableResolutions";
  }

  static const char* value(const ::micro_epsilon_scancontrol_msgs::GetAvailableResolutions&) { return value(); }
};


// service_traits::MD5Sum< ::micro_epsilon_scancontrol_msgs::GetAvailableResolutionsRequest> should match
// service_traits::MD5Sum< ::micro_epsilon_scancontrol_msgs::GetAvailableResolutions >
template<>
struct MD5Sum< ::micro_epsilon_scancontrol_msgs::GetAvailableResolutionsRequest>
{
  static const char* value()
  {
    return MD5Sum< ::micro_epsilon_scancontrol_msgs::GetAvailableResolutions >::value();
  }
  static const char* value(const ::micro_epsilon_scancontrol_msgs::GetAvailableResolutionsRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::micro_epsilon_scancontrol_msgs::GetAvailableResolutionsRequest> should match
// service_traits::DataType< ::micro_epsilon_scancontrol_msgs::GetAvailableResolutions >
template<>
struct DataType< ::micro_epsilon_scancontrol_msgs::GetAvailableResolutionsRequest>
{
  static const char* value()
  {
    return DataType< ::micro_epsilon_scancontrol_msgs::GetAvailableResolutions >::value();
  }
  static const char* value(const ::micro_epsilon_scancontrol_msgs::GetAvailableResolutionsRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::micro_epsilon_scancontrol_msgs::GetAvailableResolutionsResponse> should match
// service_traits::MD5Sum< ::micro_epsilon_scancontrol_msgs::GetAvailableResolutions >
template<>
struct MD5Sum< ::micro_epsilon_scancontrol_msgs::GetAvailableResolutionsResponse>
{
  static const char* value()
  {
    return MD5Sum< ::micro_epsilon_scancontrol_msgs::GetAvailableResolutions >::value();
  }
  static const char* value(const ::micro_epsilon_scancontrol_msgs::GetAvailableResolutionsResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::micro_epsilon_scancontrol_msgs::GetAvailableResolutionsResponse> should match
// service_traits::DataType< ::micro_epsilon_scancontrol_msgs::GetAvailableResolutions >
template<>
struct DataType< ::micro_epsilon_scancontrol_msgs::GetAvailableResolutionsResponse>
{
  static const char* value()
  {
    return DataType< ::micro_epsilon_scancontrol_msgs::GetAvailableResolutions >::value();
  }
  static const char* value(const ::micro_epsilon_scancontrol_msgs::GetAvailableResolutionsResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // MICRO_EPSILON_SCANCONTROL_MSGS_MESSAGE_GETAVAILABLERESOLUTIONS_H

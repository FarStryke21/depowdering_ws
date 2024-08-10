// Generated by gencpp from file pcl_sensor_data/StopRecording.msg
// DO NOT EDIT!


#ifndef PCL_SENSOR_DATA_MESSAGE_STOPRECORDING_H
#define PCL_SENSOR_DATA_MESSAGE_STOPRECORDING_H

#include <ros/service_traits.h>


#include <pcl_sensor_data/StopRecordingRequest.h>
#include <pcl_sensor_data/StopRecordingResponse.h>


namespace pcl_sensor_data
{

struct StopRecording
{

typedef StopRecordingRequest Request;
typedef StopRecordingResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct StopRecording
} // namespace pcl_sensor_data


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::pcl_sensor_data::StopRecording > {
  static const char* value()
  {
    return "e188e0aaebb633ed14c21333921ef038";
  }

  static const char* value(const ::pcl_sensor_data::StopRecording&) { return value(); }
};

template<>
struct DataType< ::pcl_sensor_data::StopRecording > {
  static const char* value()
  {
    return "pcl_sensor_data/StopRecording";
  }

  static const char* value(const ::pcl_sensor_data::StopRecording&) { return value(); }
};


// service_traits::MD5Sum< ::pcl_sensor_data::StopRecordingRequest> should match
// service_traits::MD5Sum< ::pcl_sensor_data::StopRecording >
template<>
struct MD5Sum< ::pcl_sensor_data::StopRecordingRequest>
{
  static const char* value()
  {
    return MD5Sum< ::pcl_sensor_data::StopRecording >::value();
  }
  static const char* value(const ::pcl_sensor_data::StopRecordingRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::pcl_sensor_data::StopRecordingRequest> should match
// service_traits::DataType< ::pcl_sensor_data::StopRecording >
template<>
struct DataType< ::pcl_sensor_data::StopRecordingRequest>
{
  static const char* value()
  {
    return DataType< ::pcl_sensor_data::StopRecording >::value();
  }
  static const char* value(const ::pcl_sensor_data::StopRecordingRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::pcl_sensor_data::StopRecordingResponse> should match
// service_traits::MD5Sum< ::pcl_sensor_data::StopRecording >
template<>
struct MD5Sum< ::pcl_sensor_data::StopRecordingResponse>
{
  static const char* value()
  {
    return MD5Sum< ::pcl_sensor_data::StopRecording >::value();
  }
  static const char* value(const ::pcl_sensor_data::StopRecordingResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::pcl_sensor_data::StopRecordingResponse> should match
// service_traits::DataType< ::pcl_sensor_data::StopRecording >
template<>
struct DataType< ::pcl_sensor_data::StopRecordingResponse>
{
  static const char* value()
  {
    return DataType< ::pcl_sensor_data::StopRecording >::value();
  }
  static const char* value(const ::pcl_sensor_data::StopRecordingResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // PCL_SENSOR_DATA_MESSAGE_STOPRECORDING_H

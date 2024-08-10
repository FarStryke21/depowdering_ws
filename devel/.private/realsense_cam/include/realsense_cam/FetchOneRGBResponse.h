// Generated by gencpp from file realsense_cam/FetchOneRGBResponse.msg
// DO NOT EDIT!


#ifndef REALSENSE_CAM_MESSAGE_FETCHONERGBRESPONSE_H
#define REALSENSE_CAM_MESSAGE_FETCHONERGBRESPONSE_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <sensor_msgs/Image.h>

namespace realsense_cam
{
template <class ContainerAllocator>
struct FetchOneRGBResponse_
{
  typedef FetchOneRGBResponse_<ContainerAllocator> Type;

  FetchOneRGBResponse_()
    : success(false)
    , reason()
    , rgb()  {
    }
  FetchOneRGBResponse_(const ContainerAllocator& _alloc)
    : success(false)
    , reason(_alloc)
    , rgb(_alloc)  {
  (void)_alloc;
    }



   typedef uint8_t _success_type;
  _success_type success;

   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _reason_type;
  _reason_type reason;

   typedef  ::sensor_msgs::Image_<ContainerAllocator>  _rgb_type;
  _rgb_type rgb;





  typedef boost::shared_ptr< ::realsense_cam::FetchOneRGBResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::realsense_cam::FetchOneRGBResponse_<ContainerAllocator> const> ConstPtr;

}; // struct FetchOneRGBResponse_

typedef ::realsense_cam::FetchOneRGBResponse_<std::allocator<void> > FetchOneRGBResponse;

typedef boost::shared_ptr< ::realsense_cam::FetchOneRGBResponse > FetchOneRGBResponsePtr;
typedef boost::shared_ptr< ::realsense_cam::FetchOneRGBResponse const> FetchOneRGBResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::realsense_cam::FetchOneRGBResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::realsense_cam::FetchOneRGBResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::realsense_cam::FetchOneRGBResponse_<ContainerAllocator1> & lhs, const ::realsense_cam::FetchOneRGBResponse_<ContainerAllocator2> & rhs)
{
  return lhs.success == rhs.success &&
    lhs.reason == rhs.reason &&
    lhs.rgb == rhs.rgb;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::realsense_cam::FetchOneRGBResponse_<ContainerAllocator1> & lhs, const ::realsense_cam::FetchOneRGBResponse_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace realsense_cam

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::realsense_cam::FetchOneRGBResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::realsense_cam::FetchOneRGBResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::realsense_cam::FetchOneRGBResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::realsense_cam::FetchOneRGBResponse_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::realsense_cam::FetchOneRGBResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::realsense_cam::FetchOneRGBResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::realsense_cam::FetchOneRGBResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ccd168e371de7ed9bbb3078068a6b679";
  }

  static const char* value(const ::realsense_cam::FetchOneRGBResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xccd168e371de7ed9ULL;
  static const uint64_t static_value2 = 0xbbb3078068a6b679ULL;
};

template<class ContainerAllocator>
struct DataType< ::realsense_cam::FetchOneRGBResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "realsense_cam/FetchOneRGBResponse";
  }

  static const char* value(const ::realsense_cam::FetchOneRGBResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::realsense_cam::FetchOneRGBResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# Success: TRUE if frame is successfully delivered.\n"
"# Reason: detail of error if not succeeded.\n"
"bool success\n"
"string reason\n"
"sensor_msgs/Image rgb\n"
"\n"
"================================================================================\n"
"MSG: sensor_msgs/Image\n"
"# This message contains an uncompressed image\n"
"# (0, 0) is at top-left corner of image\n"
"#\n"
"\n"
"Header header        # Header timestamp should be acquisition time of image\n"
"                     # Header frame_id should be optical frame of camera\n"
"                     # origin of frame should be optical center of camera\n"
"                     # +x should point to the right in the image\n"
"                     # +y should point down in the image\n"
"                     # +z should point into to plane of the image\n"
"                     # If the frame_id here and the frame_id of the CameraInfo\n"
"                     # message associated with the image conflict\n"
"                     # the behavior is undefined\n"
"\n"
"uint32 height         # image height, that is, number of rows\n"
"uint32 width          # image width, that is, number of columns\n"
"\n"
"# The legal values for encoding are in file src/image_encodings.cpp\n"
"# If you want to standardize a new string format, join\n"
"# ros-users@lists.sourceforge.net and send an email proposing a new encoding.\n"
"\n"
"string encoding       # Encoding of pixels -- channel meaning, ordering, size\n"
"                      # taken from the list of strings in include/sensor_msgs/image_encodings.h\n"
"\n"
"uint8 is_bigendian    # is this data bigendian?\n"
"uint32 step           # Full row length in bytes\n"
"uint8[] data          # actual matrix data, size is (step * rows)\n"
"\n"
"================================================================================\n"
"MSG: std_msgs/Header\n"
"# Standard metadata for higher-level stamped data types.\n"
"# This is generally used to communicate timestamped data \n"
"# in a particular coordinate frame.\n"
"# \n"
"# sequence ID: consecutively increasing ID \n"
"uint32 seq\n"
"#Two-integer timestamp that is expressed as:\n"
"# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n"
"# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n"
"# time-handling sugar is provided by the client library\n"
"time stamp\n"
"#Frame this data is associated with\n"
"string frame_id\n"
;
  }

  static const char* value(const ::realsense_cam::FetchOneRGBResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::realsense_cam::FetchOneRGBResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.success);
      stream.next(m.reason);
      stream.next(m.rgb);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct FetchOneRGBResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::realsense_cam::FetchOneRGBResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::realsense_cam::FetchOneRGBResponse_<ContainerAllocator>& v)
  {
    s << indent << "success: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.success);
    s << indent << "reason: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.reason);
    s << indent << "rgb: ";
    s << std::endl;
    Printer< ::sensor_msgs::Image_<ContainerAllocator> >::stream(s, indent + "  ", v.rgb);
  }
};

} // namespace message_operations
} // namespace ros

#endif // REALSENSE_CAM_MESSAGE_FETCHONERGBRESPONSE_H

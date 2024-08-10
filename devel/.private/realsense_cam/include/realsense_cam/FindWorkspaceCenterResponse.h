// Generated by gencpp from file realsense_cam/FindWorkspaceCenterResponse.msg
// DO NOT EDIT!


#ifndef REALSENSE_CAM_MESSAGE_FINDWORKSPACECENTERRESPONSE_H
#define REALSENSE_CAM_MESSAGE_FINDWORKSPACECENTERRESPONSE_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <geometry_msgs/PointStamped.h>

namespace realsense_cam
{
template <class ContainerAllocator>
struct FindWorkspaceCenterResponse_
{
  typedef FindWorkspaceCenterResponse_<ContainerAllocator> Type;

  FindWorkspaceCenterResponse_()
    : success(false)
    , point()
    , z_change(0.0)
    , x_change(0.0)  {
    }
  FindWorkspaceCenterResponse_(const ContainerAllocator& _alloc)
    : success(false)
    , point(_alloc)
    , z_change(0.0)
    , x_change(0.0)  {
  (void)_alloc;
    }



   typedef uint8_t _success_type;
  _success_type success;

   typedef  ::geometry_msgs::PointStamped_<ContainerAllocator>  _point_type;
  _point_type point;

   typedef double _z_change_type;
  _z_change_type z_change;

   typedef double _x_change_type;
  _x_change_type x_change;





  typedef boost::shared_ptr< ::realsense_cam::FindWorkspaceCenterResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::realsense_cam::FindWorkspaceCenterResponse_<ContainerAllocator> const> ConstPtr;

}; // struct FindWorkspaceCenterResponse_

typedef ::realsense_cam::FindWorkspaceCenterResponse_<std::allocator<void> > FindWorkspaceCenterResponse;

typedef boost::shared_ptr< ::realsense_cam::FindWorkspaceCenterResponse > FindWorkspaceCenterResponsePtr;
typedef boost::shared_ptr< ::realsense_cam::FindWorkspaceCenterResponse const> FindWorkspaceCenterResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::realsense_cam::FindWorkspaceCenterResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::realsense_cam::FindWorkspaceCenterResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::realsense_cam::FindWorkspaceCenterResponse_<ContainerAllocator1> & lhs, const ::realsense_cam::FindWorkspaceCenterResponse_<ContainerAllocator2> & rhs)
{
  return lhs.success == rhs.success &&
    lhs.point == rhs.point &&
    lhs.z_change == rhs.z_change &&
    lhs.x_change == rhs.x_change;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::realsense_cam::FindWorkspaceCenterResponse_<ContainerAllocator1> & lhs, const ::realsense_cam::FindWorkspaceCenterResponse_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace realsense_cam

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::realsense_cam::FindWorkspaceCenterResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::realsense_cam::FindWorkspaceCenterResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::realsense_cam::FindWorkspaceCenterResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::realsense_cam::FindWorkspaceCenterResponse_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::realsense_cam::FindWorkspaceCenterResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::realsense_cam::FindWorkspaceCenterResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::realsense_cam::FindWorkspaceCenterResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bd4595b8ce50ecc3768e9c5dc7e441ab";
  }

  static const char* value(const ::realsense_cam::FindWorkspaceCenterResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xbd4595b8ce50ecc3ULL;
  static const uint64_t static_value2 = 0x768e9c5dc7e441abULL;
};

template<class ContainerAllocator>
struct DataType< ::realsense_cam::FindWorkspaceCenterResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "realsense_cam/FindWorkspaceCenterResponse";
  }

  static const char* value(const ::realsense_cam::FindWorkspaceCenterResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::realsense_cam::FindWorkspaceCenterResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# Success: TRUE if frame is successfully delivered.\n"
"# Reason: detail of error if not succeeded.\n"
"bool success\n"
"geometry_msgs/PointStamped point\n"
"float64 z_change\n"
"float64 x_change\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/PointStamped\n"
"# This represents a Point with reference coordinate frame and timestamp\n"
"Header header\n"
"Point point\n"
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
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Point\n"
"# This contains the position of a point in free space\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
;
  }

  static const char* value(const ::realsense_cam::FindWorkspaceCenterResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::realsense_cam::FindWorkspaceCenterResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.success);
      stream.next(m.point);
      stream.next(m.z_change);
      stream.next(m.x_change);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct FindWorkspaceCenterResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::realsense_cam::FindWorkspaceCenterResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::realsense_cam::FindWorkspaceCenterResponse_<ContainerAllocator>& v)
  {
    s << indent << "success: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.success);
    s << indent << "point: ";
    s << std::endl;
    Printer< ::geometry_msgs::PointStamped_<ContainerAllocator> >::stream(s, indent + "  ", v.point);
    s << indent << "z_change: ";
    Printer<double>::stream(s, indent + "  ", v.z_change);
    s << indent << "x_change: ";
    Printer<double>::stream(s, indent + "  ", v.x_change);
  }
};

} // namespace message_operations
} // namespace ros

#endif // REALSENSE_CAM_MESSAGE_FINDWORKSPACECENTERRESPONSE_H

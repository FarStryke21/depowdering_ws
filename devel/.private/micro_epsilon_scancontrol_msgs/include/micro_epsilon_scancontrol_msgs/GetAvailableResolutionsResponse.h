// Generated by gencpp from file micro_epsilon_scancontrol_msgs/GetAvailableResolutionsResponse.msg
// DO NOT EDIT!


#ifndef MICRO_EPSILON_SCANCONTROL_MSGS_MESSAGE_GETAVAILABLERESOLUTIONSRESPONSE_H
#define MICRO_EPSILON_SCANCONTROL_MSGS_MESSAGE_GETAVAILABLERESOLUTIONSRESPONSE_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace micro_epsilon_scancontrol_msgs
{
template <class ContainerAllocator>
struct GetAvailableResolutionsResponse_
{
  typedef GetAvailableResolutionsResponse_<ContainerAllocator> Type;

  GetAvailableResolutionsResponse_()
    : resolutions()
    , return_code(0)  {
    }
  GetAvailableResolutionsResponse_(const ContainerAllocator& _alloc)
    : resolutions(_alloc)
    , return_code(0)  {
  (void)_alloc;
    }



   typedef std::vector<uint32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint32_t>> _resolutions_type;
  _resolutions_type resolutions;

   typedef int32_t _return_code_type;
  _return_code_type return_code;





  typedef boost::shared_ptr< ::micro_epsilon_scancontrol_msgs::GetAvailableResolutionsResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::micro_epsilon_scancontrol_msgs::GetAvailableResolutionsResponse_<ContainerAllocator> const> ConstPtr;

}; // struct GetAvailableResolutionsResponse_

typedef ::micro_epsilon_scancontrol_msgs::GetAvailableResolutionsResponse_<std::allocator<void> > GetAvailableResolutionsResponse;

typedef boost::shared_ptr< ::micro_epsilon_scancontrol_msgs::GetAvailableResolutionsResponse > GetAvailableResolutionsResponsePtr;
typedef boost::shared_ptr< ::micro_epsilon_scancontrol_msgs::GetAvailableResolutionsResponse const> GetAvailableResolutionsResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::micro_epsilon_scancontrol_msgs::GetAvailableResolutionsResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::micro_epsilon_scancontrol_msgs::GetAvailableResolutionsResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::micro_epsilon_scancontrol_msgs::GetAvailableResolutionsResponse_<ContainerAllocator1> & lhs, const ::micro_epsilon_scancontrol_msgs::GetAvailableResolutionsResponse_<ContainerAllocator2> & rhs)
{
  return lhs.resolutions == rhs.resolutions &&
    lhs.return_code == rhs.return_code;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::micro_epsilon_scancontrol_msgs::GetAvailableResolutionsResponse_<ContainerAllocator1> & lhs, const ::micro_epsilon_scancontrol_msgs::GetAvailableResolutionsResponse_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace micro_epsilon_scancontrol_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::micro_epsilon_scancontrol_msgs::GetAvailableResolutionsResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::micro_epsilon_scancontrol_msgs::GetAvailableResolutionsResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::micro_epsilon_scancontrol_msgs::GetAvailableResolutionsResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::micro_epsilon_scancontrol_msgs::GetAvailableResolutionsResponse_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::micro_epsilon_scancontrol_msgs::GetAvailableResolutionsResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::micro_epsilon_scancontrol_msgs::GetAvailableResolutionsResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::micro_epsilon_scancontrol_msgs::GetAvailableResolutionsResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "a603ff8225a17994979733ebf35285f3";
  }

  static const char* value(const ::micro_epsilon_scancontrol_msgs::GetAvailableResolutionsResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xa603ff8225a17994ULL;
  static const uint64_t static_value2 = 0x979733ebf35285f3ULL;
};

template<class ContainerAllocator>
struct DataType< ::micro_epsilon_scancontrol_msgs::GetAvailableResolutionsResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "micro_epsilon_scancontrol_msgs/GetAvailableResolutionsResponse";
  }

  static const char* value(const ::micro_epsilon_scancontrol_msgs::GetAvailableResolutionsResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::micro_epsilon_scancontrol_msgs::GetAvailableResolutionsResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "uint32[] resolutions\n"
"int32 return_code\n"
;
  }

  static const char* value(const ::micro_epsilon_scancontrol_msgs::GetAvailableResolutionsResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::micro_epsilon_scancontrol_msgs::GetAvailableResolutionsResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.resolutions);
      stream.next(m.return_code);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct GetAvailableResolutionsResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::micro_epsilon_scancontrol_msgs::GetAvailableResolutionsResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::micro_epsilon_scancontrol_msgs::GetAvailableResolutionsResponse_<ContainerAllocator>& v)
  {
    s << indent << "resolutions[]" << std::endl;
    for (size_t i = 0; i < v.resolutions.size(); ++i)
    {
      s << indent << "  resolutions[" << i << "]: ";
      Printer<uint32_t>::stream(s, indent + "  ", v.resolutions[i]);
    }
    s << indent << "return_code: ";
    Printer<int32_t>::stream(s, indent + "  ", v.return_code);
  }
};

} // namespace message_operations
} // namespace ros

#endif // MICRO_EPSILON_SCANCONTROL_MSGS_MESSAGE_GETAVAILABLERESOLUTIONSRESPONSE_H

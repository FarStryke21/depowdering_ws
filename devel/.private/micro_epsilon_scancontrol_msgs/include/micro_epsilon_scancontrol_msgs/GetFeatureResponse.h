// Generated by gencpp from file micro_epsilon_scancontrol_msgs/GetFeatureResponse.msg
// DO NOT EDIT!


#ifndef MICRO_EPSILON_SCANCONTROL_MSGS_MESSAGE_GETFEATURERESPONSE_H
#define MICRO_EPSILON_SCANCONTROL_MSGS_MESSAGE_GETFEATURERESPONSE_H


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
struct GetFeatureResponse_
{
  typedef GetFeatureResponse_<ContainerAllocator> Type;

  GetFeatureResponse_()
    : value(0)
    , return_code(0)  {
    }
  GetFeatureResponse_(const ContainerAllocator& _alloc)
    : value(0)
    , return_code(0)  {
  (void)_alloc;
    }



   typedef uint32_t _value_type;
  _value_type value;

   typedef int32_t _return_code_type;
  _return_code_type return_code;





  typedef boost::shared_ptr< ::micro_epsilon_scancontrol_msgs::GetFeatureResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::micro_epsilon_scancontrol_msgs::GetFeatureResponse_<ContainerAllocator> const> ConstPtr;

}; // struct GetFeatureResponse_

typedef ::micro_epsilon_scancontrol_msgs::GetFeatureResponse_<std::allocator<void> > GetFeatureResponse;

typedef boost::shared_ptr< ::micro_epsilon_scancontrol_msgs::GetFeatureResponse > GetFeatureResponsePtr;
typedef boost::shared_ptr< ::micro_epsilon_scancontrol_msgs::GetFeatureResponse const> GetFeatureResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::micro_epsilon_scancontrol_msgs::GetFeatureResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::micro_epsilon_scancontrol_msgs::GetFeatureResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::micro_epsilon_scancontrol_msgs::GetFeatureResponse_<ContainerAllocator1> & lhs, const ::micro_epsilon_scancontrol_msgs::GetFeatureResponse_<ContainerAllocator2> & rhs)
{
  return lhs.value == rhs.value &&
    lhs.return_code == rhs.return_code;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::micro_epsilon_scancontrol_msgs::GetFeatureResponse_<ContainerAllocator1> & lhs, const ::micro_epsilon_scancontrol_msgs::GetFeatureResponse_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace micro_epsilon_scancontrol_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::micro_epsilon_scancontrol_msgs::GetFeatureResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::micro_epsilon_scancontrol_msgs::GetFeatureResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::micro_epsilon_scancontrol_msgs::GetFeatureResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::micro_epsilon_scancontrol_msgs::GetFeatureResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::micro_epsilon_scancontrol_msgs::GetFeatureResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::micro_epsilon_scancontrol_msgs::GetFeatureResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::micro_epsilon_scancontrol_msgs::GetFeatureResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "06059ea2827d83c8150ee6fe06bbfd6a";
  }

  static const char* value(const ::micro_epsilon_scancontrol_msgs::GetFeatureResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x06059ea2827d83c8ULL;
  static const uint64_t static_value2 = 0x150ee6fe06bbfd6aULL;
};

template<class ContainerAllocator>
struct DataType< ::micro_epsilon_scancontrol_msgs::GetFeatureResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "micro_epsilon_scancontrol_msgs/GetFeatureResponse";
  }

  static const char* value(const ::micro_epsilon_scancontrol_msgs::GetFeatureResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::micro_epsilon_scancontrol_msgs::GetFeatureResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "uint32 value\n"
"int32 return_code\n"
;
  }

  static const char* value(const ::micro_epsilon_scancontrol_msgs::GetFeatureResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::micro_epsilon_scancontrol_msgs::GetFeatureResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.value);
      stream.next(m.return_code);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct GetFeatureResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::micro_epsilon_scancontrol_msgs::GetFeatureResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::micro_epsilon_scancontrol_msgs::GetFeatureResponse_<ContainerAllocator>& v)
  {
    s << indent << "value: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.value);
    s << indent << "return_code: ";
    Printer<int32_t>::stream(s, indent + "  ", v.return_code);
  }
};

} // namespace message_operations
} // namespace ros

#endif // MICRO_EPSILON_SCANCONTROL_MSGS_MESSAGE_GETFEATURERESPONSE_H

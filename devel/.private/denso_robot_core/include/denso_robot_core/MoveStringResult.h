// Generated by gencpp from file denso_robot_core/MoveStringResult.msg
// DO NOT EDIT!


#ifndef DENSO_ROBOT_CORE_MESSAGE_MOVESTRINGRESULT_H
#define DENSO_ROBOT_CORE_MESSAGE_MOVESTRINGRESULT_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace denso_robot_core
{
template <class ContainerAllocator>
struct MoveStringResult_
{
  typedef MoveStringResult_<ContainerAllocator> Type;

  MoveStringResult_()
    : HRESULT(0)  {
    }
  MoveStringResult_(const ContainerAllocator& _alloc)
    : HRESULT(0)  {
  (void)_alloc;
    }



   typedef int32_t _HRESULT_type;
  _HRESULT_type HRESULT;





  typedef boost::shared_ptr< ::denso_robot_core::MoveStringResult_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::denso_robot_core::MoveStringResult_<ContainerAllocator> const> ConstPtr;

}; // struct MoveStringResult_

typedef ::denso_robot_core::MoveStringResult_<std::allocator<void> > MoveStringResult;

typedef boost::shared_ptr< ::denso_robot_core::MoveStringResult > MoveStringResultPtr;
typedef boost::shared_ptr< ::denso_robot_core::MoveStringResult const> MoveStringResultConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::denso_robot_core::MoveStringResult_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::denso_robot_core::MoveStringResult_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::denso_robot_core::MoveStringResult_<ContainerAllocator1> & lhs, const ::denso_robot_core::MoveStringResult_<ContainerAllocator2> & rhs)
{
  return lhs.HRESULT == rhs.HRESULT;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::denso_robot_core::MoveStringResult_<ContainerAllocator1> & lhs, const ::denso_robot_core::MoveStringResult_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace denso_robot_core

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::denso_robot_core::MoveStringResult_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::denso_robot_core::MoveStringResult_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::denso_robot_core::MoveStringResult_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::denso_robot_core::MoveStringResult_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::denso_robot_core::MoveStringResult_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::denso_robot_core::MoveStringResult_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::denso_robot_core::MoveStringResult_<ContainerAllocator> >
{
  static const char* value()
  {
    return "0e844160689200392730e0ad31896226";
  }

  static const char* value(const ::denso_robot_core::MoveStringResult_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x0e84416068920039ULL;
  static const uint64_t static_value2 = 0x2730e0ad31896226ULL;
};

template<class ContainerAllocator>
struct DataType< ::denso_robot_core::MoveStringResult_<ContainerAllocator> >
{
  static const char* value()
  {
    return "denso_robot_core/MoveStringResult";
  }

  static const char* value(const ::denso_robot_core::MoveStringResult_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::denso_robot_core::MoveStringResult_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n"
"int32 HRESULT\n"
;
  }

  static const char* value(const ::denso_robot_core::MoveStringResult_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::denso_robot_core::MoveStringResult_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.HRESULT);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct MoveStringResult_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::denso_robot_core::MoveStringResult_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::denso_robot_core::MoveStringResult_<ContainerAllocator>& v)
  {
    s << indent << "HRESULT: ";
    Printer<int32_t>::stream(s, indent + "  ", v.HRESULT);
  }
};

} // namespace message_operations
} // namespace ros

#endif // DENSO_ROBOT_CORE_MESSAGE_MOVESTRINGRESULT_H

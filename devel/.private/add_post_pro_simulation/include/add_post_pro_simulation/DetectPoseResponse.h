// Generated by gencpp from file add_post_pro_simulation/DetectPoseResponse.msg
// DO NOT EDIT!


#ifndef ADD_POST_PRO_SIMULATION_MESSAGE_DETECTPOSERESPONSE_H
#define ADD_POST_PRO_SIMULATION_MESSAGE_DETECTPOSERESPONSE_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace add_post_pro_simulation
{
template <class ContainerAllocator>
struct DetectPoseResponse_
{
  typedef DetectPoseResponse_<ContainerAllocator> Type;

  DetectPoseResponse_()
    : x(0.0)
    , y(0.0)
    , z(0.0)  {
    }
  DetectPoseResponse_(const ContainerAllocator& _alloc)
    : x(0.0)
    , y(0.0)
    , z(0.0)  {
  (void)_alloc;
    }



   typedef double _x_type;
  _x_type x;

   typedef double _y_type;
  _y_type y;

   typedef double _z_type;
  _z_type z;





  typedef boost::shared_ptr< ::add_post_pro_simulation::DetectPoseResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::add_post_pro_simulation::DetectPoseResponse_<ContainerAllocator> const> ConstPtr;

}; // struct DetectPoseResponse_

typedef ::add_post_pro_simulation::DetectPoseResponse_<std::allocator<void> > DetectPoseResponse;

typedef boost::shared_ptr< ::add_post_pro_simulation::DetectPoseResponse > DetectPoseResponsePtr;
typedef boost::shared_ptr< ::add_post_pro_simulation::DetectPoseResponse const> DetectPoseResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::add_post_pro_simulation::DetectPoseResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::add_post_pro_simulation::DetectPoseResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::add_post_pro_simulation::DetectPoseResponse_<ContainerAllocator1> & lhs, const ::add_post_pro_simulation::DetectPoseResponse_<ContainerAllocator2> & rhs)
{
  return lhs.x == rhs.x &&
    lhs.y == rhs.y &&
    lhs.z == rhs.z;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::add_post_pro_simulation::DetectPoseResponse_<ContainerAllocator1> & lhs, const ::add_post_pro_simulation::DetectPoseResponse_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace add_post_pro_simulation

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::add_post_pro_simulation::DetectPoseResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::add_post_pro_simulation::DetectPoseResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::add_post_pro_simulation::DetectPoseResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::add_post_pro_simulation::DetectPoseResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::add_post_pro_simulation::DetectPoseResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::add_post_pro_simulation::DetectPoseResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::add_post_pro_simulation::DetectPoseResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "4a842b65f413084dc2b10fb484ea7f17";
  }

  static const char* value(const ::add_post_pro_simulation::DetectPoseResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x4a842b65f413084dULL;
  static const uint64_t static_value2 = 0xc2b10fb484ea7f17ULL;
};

template<class ContainerAllocator>
struct DataType< ::add_post_pro_simulation::DetectPoseResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "add_post_pro_simulation/DetectPoseResponse";
  }

  static const char* value(const ::add_post_pro_simulation::DetectPoseResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::add_post_pro_simulation::DetectPoseResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float64 x\n"
"float64 y\n"
"float64 z\n"
;
  }

  static const char* value(const ::add_post_pro_simulation::DetectPoseResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::add_post_pro_simulation::DetectPoseResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.x);
      stream.next(m.y);
      stream.next(m.z);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct DetectPoseResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::add_post_pro_simulation::DetectPoseResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::add_post_pro_simulation::DetectPoseResponse_<ContainerAllocator>& v)
  {
    s << indent << "x: ";
    Printer<double>::stream(s, indent + "  ", v.x);
    s << indent << "y: ";
    Printer<double>::stream(s, indent + "  ", v.y);
    s << indent << "z: ";
    Printer<double>::stream(s, indent + "  ", v.z);
  }
};

} // namespace message_operations
} // namespace ros

#endif // ADD_POST_PRO_SIMULATION_MESSAGE_DETECTPOSERESPONSE_H

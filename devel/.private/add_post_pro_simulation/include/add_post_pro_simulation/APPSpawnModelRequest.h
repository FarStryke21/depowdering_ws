// Generated by gencpp from file add_post_pro_simulation/APPSpawnModelRequest.msg
// DO NOT EDIT!


#ifndef ADD_POST_PRO_SIMULATION_MESSAGE_APPSPAWNMODELREQUEST_H
#define ADD_POST_PRO_SIMULATION_MESSAGE_APPSPAWNMODELREQUEST_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <geometry_msgs/Pose.h>

namespace add_post_pro_simulation
{
template <class ContainerAllocator>
struct APPSpawnModelRequest_
{
  typedef APPSpawnModelRequest_<ContainerAllocator> Type;

  APPSpawnModelRequest_()
    : model_name()
    , instance_name()
    , turntable(false)
    , pose()
    , parent_frame()  {
    }
  APPSpawnModelRequest_(const ContainerAllocator& _alloc)
    : model_name(_alloc)
    , instance_name(_alloc)
    , turntable(false)
    , pose(_alloc)
    , parent_frame(_alloc)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _model_name_type;
  _model_name_type model_name;

   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _instance_name_type;
  _instance_name_type instance_name;

   typedef uint8_t _turntable_type;
  _turntable_type turntable;

   typedef  ::geometry_msgs::Pose_<ContainerAllocator>  _pose_type;
  _pose_type pose;

   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _parent_frame_type;
  _parent_frame_type parent_frame;





  typedef boost::shared_ptr< ::add_post_pro_simulation::APPSpawnModelRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::add_post_pro_simulation::APPSpawnModelRequest_<ContainerAllocator> const> ConstPtr;

}; // struct APPSpawnModelRequest_

typedef ::add_post_pro_simulation::APPSpawnModelRequest_<std::allocator<void> > APPSpawnModelRequest;

typedef boost::shared_ptr< ::add_post_pro_simulation::APPSpawnModelRequest > APPSpawnModelRequestPtr;
typedef boost::shared_ptr< ::add_post_pro_simulation::APPSpawnModelRequest const> APPSpawnModelRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::add_post_pro_simulation::APPSpawnModelRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::add_post_pro_simulation::APPSpawnModelRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::add_post_pro_simulation::APPSpawnModelRequest_<ContainerAllocator1> & lhs, const ::add_post_pro_simulation::APPSpawnModelRequest_<ContainerAllocator2> & rhs)
{
  return lhs.model_name == rhs.model_name &&
    lhs.instance_name == rhs.instance_name &&
    lhs.turntable == rhs.turntable &&
    lhs.pose == rhs.pose &&
    lhs.parent_frame == rhs.parent_frame;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::add_post_pro_simulation::APPSpawnModelRequest_<ContainerAllocator1> & lhs, const ::add_post_pro_simulation::APPSpawnModelRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace add_post_pro_simulation

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::add_post_pro_simulation::APPSpawnModelRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::add_post_pro_simulation::APPSpawnModelRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::add_post_pro_simulation::APPSpawnModelRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::add_post_pro_simulation::APPSpawnModelRequest_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::add_post_pro_simulation::APPSpawnModelRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::add_post_pro_simulation::APPSpawnModelRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::add_post_pro_simulation::APPSpawnModelRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "76a460d134ffd162f77fd5071072d4e7";
  }

  static const char* value(const ::add_post_pro_simulation::APPSpawnModelRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x76a460d134ffd162ULL;
  static const uint64_t static_value2 = 0xf77fd5071072d4e7ULL;
};

template<class ContainerAllocator>
struct DataType< ::add_post_pro_simulation::APPSpawnModelRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "add_post_pro_simulation/APPSpawnModelRequest";
  }

  static const char* value(const ::add_post_pro_simulation::APPSpawnModelRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::add_post_pro_simulation::APPSpawnModelRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "string model_name\n"
"string instance_name\n"
"bool turntable\n"
"geometry_msgs/Pose pose\n"
"string parent_frame\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Pose\n"
"# A representation of pose in free space, composed of position and orientation. \n"
"Point position\n"
"Quaternion orientation\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Point\n"
"# This contains the position of a point in free space\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Quaternion\n"
"# This represents an orientation in free space in quaternion form.\n"
"\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"float64 w\n"
;
  }

  static const char* value(const ::add_post_pro_simulation::APPSpawnModelRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::add_post_pro_simulation::APPSpawnModelRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.model_name);
      stream.next(m.instance_name);
      stream.next(m.turntable);
      stream.next(m.pose);
      stream.next(m.parent_frame);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct APPSpawnModelRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::add_post_pro_simulation::APPSpawnModelRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::add_post_pro_simulation::APPSpawnModelRequest_<ContainerAllocator>& v)
  {
    s << indent << "model_name: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.model_name);
    s << indent << "instance_name: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.instance_name);
    s << indent << "turntable: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.turntable);
    s << indent << "pose: ";
    s << std::endl;
    Printer< ::geometry_msgs::Pose_<ContainerAllocator> >::stream(s, indent + "  ", v.pose);
    s << indent << "parent_frame: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.parent_frame);
  }
};

} // namespace message_operations
} // namespace ros

#endif // ADD_POST_PRO_SIMULATION_MESSAGE_APPSPAWNMODELREQUEST_H

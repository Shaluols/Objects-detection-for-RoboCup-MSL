// Generated by gencpp from file nubot_common/MotorInfo.msg
// DO NOT EDIT!


#ifndef NUBOT_COMMON_MESSAGE_MOTORINFO_H
#define NUBOT_COMMON_MESSAGE_MOTORINFO_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace nubot_common
{
template <class ContainerAllocator>
struct MotorInfo_
{
  typedef MotorInfo_<ContainerAllocator> Type;

  MotorInfo_()
    : header()
    , motordata()  {
      motordata.assign(0);
  }
  MotorInfo_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , motordata()  {
  (void)_alloc;
      motordata.assign(0);
  }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef boost::array<uint32_t, 4>  _motordata_type;
  _motordata_type motordata;




  typedef boost::shared_ptr< ::nubot_common::MotorInfo_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::nubot_common::MotorInfo_<ContainerAllocator> const> ConstPtr;

}; // struct MotorInfo_

typedef ::nubot_common::MotorInfo_<std::allocator<void> > MotorInfo;

typedef boost::shared_ptr< ::nubot_common::MotorInfo > MotorInfoPtr;
typedef boost::shared_ptr< ::nubot_common::MotorInfo const> MotorInfoConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::nubot_common::MotorInfo_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::nubot_common::MotorInfo_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace nubot_common

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': True}
// {'geometry_msgs': ['/opt/ros/jade/share/geometry_msgs/cmake/../msg'], 'nubot_common': ['/home/ubuntu/libfreenect2-master/src/nubot_common/msgs'], 'std_msgs': ['/opt/ros/jade/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::nubot_common::MotorInfo_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::nubot_common::MotorInfo_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::nubot_common::MotorInfo_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::nubot_common::MotorInfo_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::nubot_common::MotorInfo_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::nubot_common::MotorInfo_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::nubot_common::MotorInfo_<ContainerAllocator> >
{
  static const char* value()
  {
    return "a75d984a29f54312561eb327c067e655";
  }

  static const char* value(const ::nubot_common::MotorInfo_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xa75d984a29f54312ULL;
  static const uint64_t static_value2 = 0x561eb327c067e655ULL;
};

template<class ContainerAllocator>
struct DataType< ::nubot_common::MotorInfo_<ContainerAllocator> >
{
  static const char* value()
  {
    return "nubot_common/MotorInfo";
  }

  static const char* value(const ::nubot_common::MotorInfo_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::nubot_common::MotorInfo_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n\
uint32[4] motordata\n\
\n\
\n\
================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n\
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
";
  }

  static const char* value(const ::nubot_common::MotorInfo_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::nubot_common::MotorInfo_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.motordata);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct MotorInfo_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::nubot_common::MotorInfo_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::nubot_common::MotorInfo_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "motordata[]" << std::endl;
    for (size_t i = 0; i < v.motordata.size(); ++i)
    {
      s << indent << "  motordata[" << i << "]: ";
      Printer<uint32_t>::stream(s, indent + "  ", v.motordata[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // NUBOT_COMMON_MESSAGE_MOTORINFO_H
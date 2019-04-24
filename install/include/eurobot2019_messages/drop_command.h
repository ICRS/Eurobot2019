// Generated by gencpp from file eurobot2019_messages/drop_command.msg
// DO NOT EDIT!


#ifndef EUROBOT2019_MESSAGES_MESSAGE_DROP_COMMAND_H
#define EUROBOT2019_MESSAGES_MESSAGE_DROP_COMMAND_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace eurobot2019_messages
{
template <class ContainerAllocator>
struct drop_command_
{
  typedef drop_command_<ContainerAllocator> Type;

  drop_command_()
    : left(false)
    , right(false)
    , middle(false)  {
    }
  drop_command_(const ContainerAllocator& _alloc)
    : left(false)
    , right(false)
    , middle(false)  {
  (void)_alloc;
    }



   typedef uint8_t _left_type;
  _left_type left;

   typedef uint8_t _right_type;
  _right_type right;

   typedef uint8_t _middle_type;
  _middle_type middle;





  typedef boost::shared_ptr< ::eurobot2019_messages::drop_command_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::eurobot2019_messages::drop_command_<ContainerAllocator> const> ConstPtr;

}; // struct drop_command_

typedef ::eurobot2019_messages::drop_command_<std::allocator<void> > drop_command;

typedef boost::shared_ptr< ::eurobot2019_messages::drop_command > drop_commandPtr;
typedef boost::shared_ptr< ::eurobot2019_messages::drop_command const> drop_commandConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::eurobot2019_messages::drop_command_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::eurobot2019_messages::drop_command_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace eurobot2019_messages

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/melodic/share/std_msgs/cmake/../msg'], 'eurobot2019_messages': ['/home/nickick/Eurobot2019/src/eurobot2019_messages/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::eurobot2019_messages::drop_command_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::eurobot2019_messages::drop_command_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::eurobot2019_messages::drop_command_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::eurobot2019_messages::drop_command_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::eurobot2019_messages::drop_command_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::eurobot2019_messages::drop_command_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::eurobot2019_messages::drop_command_<ContainerAllocator> >
{
  static const char* value()
  {
    return "c498bd496c426b7314def7449c775a44";
  }

  static const char* value(const ::eurobot2019_messages::drop_command_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xc498bd496c426b73ULL;
  static const uint64_t static_value2 = 0x14def7449c775a44ULL;
};

template<class ContainerAllocator>
struct DataType< ::eurobot2019_messages::drop_command_<ContainerAllocator> >
{
  static const char* value()
  {
    return "eurobot2019_messages/drop_command";
  }

  static const char* value(const ::eurobot2019_messages::drop_command_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::eurobot2019_messages::drop_command_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# Which tower to drop from\n"
"# 0 is idle, 1 is unload\n"
"bool left\n"
"bool right\n"
"bool middle\n"
;
  }

  static const char* value(const ::eurobot2019_messages::drop_command_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::eurobot2019_messages::drop_command_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.left);
      stream.next(m.right);
      stream.next(m.middle);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct drop_command_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::eurobot2019_messages::drop_command_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::eurobot2019_messages::drop_command_<ContainerAllocator>& v)
  {
    s << indent << "left: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.left);
    s << indent << "right: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.right);
    s << indent << "middle: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.middle);
  }
};

} // namespace message_operations
} // namespace ros

#endif // EUROBOT2019_MESSAGES_MESSAGE_DROP_COMMAND_H
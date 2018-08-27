// Generated by gencpp from file joystick/Joystick.msg
// DO NOT EDIT!


#ifndef JOYSTICK_MESSAGE_JOYSTICK_H
#define JOYSTICK_MESSAGE_JOYSTICK_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace joystick
{
template <class ContainerAllocator>
struct Joystick_
{
  typedef Joystick_<ContainerAllocator> Type;

  Joystick_()
    : time()
    , event(0)  {
    }
  Joystick_(const ContainerAllocator& _alloc)
    : time()
    , event(0)  {
  (void)_alloc;
    }



   typedef ros::Time _time_type;
  _time_type time;

   typedef int8_t _event_type;
  _event_type event;





  typedef boost::shared_ptr< ::joystick::Joystick_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::joystick::Joystick_<ContainerAllocator> const> ConstPtr;

}; // struct Joystick_

typedef ::joystick::Joystick_<std::allocator<void> > Joystick;

typedef boost::shared_ptr< ::joystick::Joystick > JoystickPtr;
typedef boost::shared_ptr< ::joystick::Joystick const> JoystickConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::joystick::Joystick_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::joystick::Joystick_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace joystick

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'joystick': ['/home/hl/Src/CLionProjects/RxROS/src/joystick/msg'], 'std_msgs': ['/opt/ros/melodic/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::joystick::Joystick_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::joystick::Joystick_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::joystick::Joystick_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::joystick::Joystick_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::joystick::Joystick_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::joystick::Joystick_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::joystick::Joystick_<ContainerAllocator> >
{
  static const char* value()
  {
    return "e5fbe2cc6f38678d510728a2dcb2ff75";
  }

  static const char* value(const ::joystick::Joystick_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xe5fbe2cc6f38678dULL;
  static const uint64_t static_value2 = 0x510728a2dcb2ff75ULL;
};

template<class ContainerAllocator>
struct DataType< ::joystick::Joystick_<ContainerAllocator> >
{
  static const char* value()
  {
    return "joystick/Joystick";
  }

  static const char* value(const ::joystick::Joystick_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::joystick::Joystick_<ContainerAllocator> >
{
  static const char* value()
  {
    return "time time\n\
int8 event\n\
";
  }

  static const char* value(const ::joystick::Joystick_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::joystick::Joystick_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.time);
      stream.next(m.event);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Joystick_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::joystick::Joystick_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::joystick::Joystick_<ContainerAllocator>& v)
  {
    s << indent << "time: ";
    Printer<ros::Time>::stream(s, indent + "  ", v.time);
    s << indent << "event: ";
    Printer<int8_t>::stream(s, indent + "  ", v.event);
  }
};

} // namespace message_operations
} // namespace ros

#endif // JOYSTICK_MESSAGE_JOYSTICK_H

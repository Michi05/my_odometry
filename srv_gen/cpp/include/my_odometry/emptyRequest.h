/* Auto-generated by genmsg_cpp for file /home/r00t/ros_workspace/my_odometry/srv/emptyRequest.srv */
#ifndef MY_ODOMETRY_SERVICE_EMPTYREQUEST_H
#define MY_ODOMETRY_SERVICE_EMPTYREQUEST_H
#include <string>
#include <vector>
#include <map>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/time.h"

#include "ros/macros.h"

#include "ros/assert.h"

#include "ros/service_traits.h"



#include "my_odometry/odom_answer.h"

namespace my_odometry
{
template <class ContainerAllocator>
struct emptyRequestRequest_ {
  typedef emptyRequestRequest_<ContainerAllocator> Type;

  emptyRequestRequest_()
  {
  }

  emptyRequestRequest_(const ContainerAllocator& _alloc)
  {
  }


  typedef boost::shared_ptr< ::my_odometry::emptyRequestRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::my_odometry::emptyRequestRequest_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct emptyRequestRequest
typedef  ::my_odometry::emptyRequestRequest_<std::allocator<void> > emptyRequestRequest;

typedef boost::shared_ptr< ::my_odometry::emptyRequestRequest> emptyRequestRequestPtr;
typedef boost::shared_ptr< ::my_odometry::emptyRequestRequest const> emptyRequestRequestConstPtr;


template <class ContainerAllocator>
struct emptyRequestResponse_ {
  typedef emptyRequestResponse_<ContainerAllocator> Type;

  emptyRequestResponse_()
  : answer()
  {
  }

  emptyRequestResponse_(const ContainerAllocator& _alloc)
  : answer(_alloc)
  {
  }

  typedef  ::my_odometry::odom_answer_<ContainerAllocator>  _answer_type;
   ::my_odometry::odom_answer_<ContainerAllocator>  answer;


  typedef boost::shared_ptr< ::my_odometry::emptyRequestResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::my_odometry::emptyRequestResponse_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct emptyRequestResponse
typedef  ::my_odometry::emptyRequestResponse_<std::allocator<void> > emptyRequestResponse;

typedef boost::shared_ptr< ::my_odometry::emptyRequestResponse> emptyRequestResponsePtr;
typedef boost::shared_ptr< ::my_odometry::emptyRequestResponse const> emptyRequestResponseConstPtr;

struct emptyRequest
{

typedef emptyRequestRequest Request;
typedef emptyRequestResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;
}; // struct emptyRequest
} // namespace my_odometry

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::my_odometry::emptyRequestRequest_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::my_odometry::emptyRequestRequest_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::my_odometry::emptyRequestRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "d41d8cd98f00b204e9800998ecf8427e";
  }

  static const char* value(const  ::my_odometry::emptyRequestRequest_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xd41d8cd98f00b204ULL;
  static const uint64_t static_value2 = 0xe9800998ecf8427eULL;
};

template<class ContainerAllocator>
struct DataType< ::my_odometry::emptyRequestRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "my_odometry/emptyRequestRequest";
  }

  static const char* value(const  ::my_odometry::emptyRequestRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::my_odometry::emptyRequestRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "\n\
\n\
";
  }

  static const char* value(const  ::my_odometry::emptyRequestRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::my_odometry::emptyRequestRequest_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros


namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::my_odometry::emptyRequestResponse_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::my_odometry::emptyRequestResponse_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::my_odometry::emptyRequestResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "0fd54208cf1a13d4feef7d72c284d8d1";
  }

  static const char* value(const  ::my_odometry::emptyRequestResponse_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x0fd54208cf1a13d4ULL;
  static const uint64_t static_value2 = 0xfeef7d72c284d8d1ULL;
};

template<class ContainerAllocator>
struct DataType< ::my_odometry::emptyRequestResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "my_odometry/emptyRequestResponse";
  }

  static const char* value(const  ::my_odometry::emptyRequestResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::my_odometry::emptyRequestResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "my_odometry/odom_answer answer\n\
\n\
\n\
\n\
================================================================================\n\
MSG: my_odometry/odom_answer\n\
## Common answer message for the odometry package\n\
##the services are suppossed to answer with this\n\
##info everytime.\n\
\n\
#   Global/Relative transform/odometry messages:\n\
# Transform frame from tf2 package\n\
tf2_msgs/TFMessage globalTF\n\
tf2_msgs/TFMessage relativeTF\n\
# System's odometry messages\n\
nav_msgs/Odometry globalOdometry\n\
nav_msgs/Odometry relativeOdometry\n\
\n\
# Code number for the status of the whole algorithm\n\
int64 statusCode\n\
\n\
\n\
================================================================================\n\
MSG: tf2_msgs/TFMessage\n\
geometry_msgs/TransformStamped[] transforms\n\
\n\
================================================================================\n\
MSG: geometry_msgs/TransformStamped\n\
# This expresses a transform from coordinate frame header.frame_id\n\
# to the coordinate frame child_frame_id\n\
#\n\
# This message is mostly used by the \n\
# <a href=\"http://www.ros.org/wiki/tf\">tf</a> package. \n\
# See it's documentation for more information.\n\
\n\
Header header\n\
string child_frame_id # the frame id of the child frame\n\
Transform transform\n\
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
# * stamp.secs: seconds (stamp_secs) since epoch\n\
# * stamp.nsecs: nanoseconds since stamp_secs\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Transform\n\
# This represents the transform between two coordinate frames in free space.\n\
\n\
Vector3 translation\n\
Quaternion rotation\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Vector3\n\
# This represents a vector in free space. \n\
\n\
float64 x\n\
float64 y\n\
float64 z\n\
================================================================================\n\
MSG: geometry_msgs/Quaternion\n\
# This represents an orientation in free space in quaternion form.\n\
\n\
float64 x\n\
float64 y\n\
float64 z\n\
float64 w\n\
\n\
================================================================================\n\
MSG: nav_msgs/Odometry\n\
# This represents an estimate of a position and velocity in free space.  \n\
# The pose in this message should be specified in the coordinate frame given by header.frame_id.\n\
# The twist in this message should be specified in the coordinate frame given by the child_frame_id\n\
Header header\n\
string child_frame_id\n\
geometry_msgs/PoseWithCovariance pose\n\
geometry_msgs/TwistWithCovariance twist\n\
\n\
================================================================================\n\
MSG: geometry_msgs/PoseWithCovariance\n\
# This represents a pose in free space with uncertainty.\n\
\n\
Pose pose\n\
\n\
# Row-major representation of the 6x6 covariance matrix\n\
# The orientation parameters use a fixed-axis representation.\n\
# In order, the parameters are:\n\
# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)\n\
float64[36] covariance\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Pose\n\
# A representation of pose in free space, composed of postion and orientation. \n\
Point position\n\
Quaternion orientation\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Point\n\
# This contains the position of a point in free space\n\
float64 x\n\
float64 y\n\
float64 z\n\
\n\
================================================================================\n\
MSG: geometry_msgs/TwistWithCovariance\n\
# This expresses velocity in free space with uncertianty.\n\
\n\
Twist twist\n\
\n\
# Row-major representation of the 6x6 covariance matrix\n\
# The orientation parameters use a fixed-axis representation.\n\
# In order, the parameters are:\n\
# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)\n\
float64[36] covariance\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Twist\n\
# This expresses velocity in free space broken into it's linear and angular parts. \n\
Vector3  linear\n\
Vector3  angular\n\
\n\
";
  }

  static const char* value(const  ::my_odometry::emptyRequestResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::my_odometry::emptyRequestRequest_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct emptyRequestRequest_
} // namespace serialization
} // namespace ros


namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::my_odometry::emptyRequestResponse_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.answer);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct emptyRequestResponse_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace service_traits
{
template<>
struct MD5Sum<my_odometry::emptyRequest> {
  static const char* value() 
  {
    return "0fd54208cf1a13d4feef7d72c284d8d1";
  }

  static const char* value(const my_odometry::emptyRequest&) { return value(); } 
};

template<>
struct DataType<my_odometry::emptyRequest> {
  static const char* value() 
  {
    return "my_odometry/emptyRequest";
  }

  static const char* value(const my_odometry::emptyRequest&) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<my_odometry::emptyRequestRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "0fd54208cf1a13d4feef7d72c284d8d1";
  }

  static const char* value(const my_odometry::emptyRequestRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<my_odometry::emptyRequestRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "my_odometry/emptyRequest";
  }

  static const char* value(const my_odometry::emptyRequestRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<my_odometry::emptyRequestResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "0fd54208cf1a13d4feef7d72c284d8d1";
  }

  static const char* value(const my_odometry::emptyRequestResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<my_odometry::emptyRequestResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "my_odometry/emptyRequest";
  }

  static const char* value(const my_odometry::emptyRequestResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace service_traits
} // namespace ros

#endif // MY_ODOMETRY_SERVICE_EMPTYREQUEST_H


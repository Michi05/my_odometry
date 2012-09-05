"""autogenerated by genpy from my_odometry/emptyRequestRequest.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class emptyRequestRequest(genpy.Message):
  _md5sum = "d41d8cd98f00b204e9800998ecf8427e"
  _type = "my_odometry/emptyRequestRequest"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """

"""
  __slots__ = []
  _slot_types = []

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(emptyRequestRequest, self).__init__(*args, **kwds)

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      pass
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      end = 0
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      pass
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      end = 0
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
"""autogenerated by genpy from my_odometry/emptyRequestResponse.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import nav_msgs.msg
import geometry_msgs.msg
import my_odometry.msg
import tf2_msgs.msg
import std_msgs.msg

class emptyRequestResponse(genpy.Message):
  _md5sum = "0fd54208cf1a13d4feef7d72c284d8d1"
  _type = "my_odometry/emptyRequestResponse"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """my_odometry/odom_answer answer



================================================================================
MSG: my_odometry/odom_answer
## Common answer message for the odometry package
##the services are suppossed to answer with this
##info everytime.

#   Global/Relative transform/odometry messages:
# Transform frame from tf2 package
tf2_msgs/TFMessage globalTF
tf2_msgs/TFMessage relativeTF
# System's odometry messages
nav_msgs/Odometry globalOdometry
nav_msgs/Odometry relativeOdometry

# Code number for the status of the whole algorithm
int64 statusCode


================================================================================
MSG: tf2_msgs/TFMessage
geometry_msgs/TransformStamped[] transforms

================================================================================
MSG: geometry_msgs/TransformStamped
# This expresses a transform from coordinate frame header.frame_id
# to the coordinate frame child_frame_id
#
# This message is mostly used by the 
# <a href="http://www.ros.org/wiki/tf">tf</a> package. 
# See it's documentation for more information.

Header header
string child_frame_id # the frame id of the child frame
Transform transform

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.secs: seconds (stamp_secs) since epoch
# * stamp.nsecs: nanoseconds since stamp_secs
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

================================================================================
MSG: geometry_msgs/Transform
# This represents the transform between two coordinate frames in free space.

Vector3 translation
Quaternion rotation

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 

float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

================================================================================
MSG: nav_msgs/Odometry
# This represents an estimate of a position and velocity in free space.  
# The pose in this message should be specified in the coordinate frame given by header.frame_id.
# The twist in this message should be specified in the coordinate frame given by the child_frame_id
Header header
string child_frame_id
geometry_msgs/PoseWithCovariance pose
geometry_msgs/TwistWithCovariance twist

================================================================================
MSG: geometry_msgs/PoseWithCovariance
# This represents a pose in free space with uncertainty.

Pose pose

# Row-major representation of the 6x6 covariance matrix
# The orientation parameters use a fixed-axis representation.
# In order, the parameters are:
# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
float64[36] covariance

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of postion and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/TwistWithCovariance
# This expresses velocity in free space with uncertianty.

Twist twist

# Row-major representation of the 6x6 covariance matrix
# The orientation parameters use a fixed-axis representation.
# In order, the parameters are:
# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
float64[36] covariance

================================================================================
MSG: geometry_msgs/Twist
# This expresses velocity in free space broken into it's linear and angular parts. 
Vector3  linear
Vector3  angular

"""
  __slots__ = ['answer']
  _slot_types = ['my_odometry/odom_answer']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       answer

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(emptyRequestResponse, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.answer is None:
        self.answer = my_odometry.msg.odom_answer()
    else:
      self.answer = my_odometry.msg.odom_answer()

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      length = len(self.answer.globalTF.transforms)
      buff.write(_struct_I.pack(length))
      for val1 in self.answer.globalTF.transforms:
        _v1 = val1.header
        buff.write(_struct_I.pack(_v1.seq))
        _v2 = _v1.stamp
        _x = _v2
        buff.write(_struct_2I.pack(_x.secs, _x.nsecs))
        _x = _v1.frame_id
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.pack('<I%ss'%length, length, _x))
        _x = val1.child_frame_id
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.pack('<I%ss'%length, length, _x))
        _v3 = val1.transform
        _v4 = _v3.translation
        _x = _v4
        buff.write(_struct_3d.pack(_x.x, _x.y, _x.z))
        _v5 = _v3.rotation
        _x = _v5
        buff.write(_struct_4d.pack(_x.x, _x.y, _x.z, _x.w))
      length = len(self.answer.relativeTF.transforms)
      buff.write(_struct_I.pack(length))
      for val1 in self.answer.relativeTF.transforms:
        _v6 = val1.header
        buff.write(_struct_I.pack(_v6.seq))
        _v7 = _v6.stamp
        _x = _v7
        buff.write(_struct_2I.pack(_x.secs, _x.nsecs))
        _x = _v6.frame_id
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.pack('<I%ss'%length, length, _x))
        _x = val1.child_frame_id
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.pack('<I%ss'%length, length, _x))
        _v8 = val1.transform
        _v9 = _v8.translation
        _x = _v9
        buff.write(_struct_3d.pack(_x.x, _x.y, _x.z))
        _v10 = _v8.rotation
        _x = _v10
        buff.write(_struct_4d.pack(_x.x, _x.y, _x.z, _x.w))
      _x = self
      buff.write(_struct_3I.pack(_x.answer.globalOdometry.header.seq, _x.answer.globalOdometry.header.stamp.secs, _x.answer.globalOdometry.header.stamp.nsecs))
      _x = self.answer.globalOdometry.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self.answer.globalOdometry.child_frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_7d.pack(_x.answer.globalOdometry.pose.pose.position.x, _x.answer.globalOdometry.pose.pose.position.y, _x.answer.globalOdometry.pose.pose.position.z, _x.answer.globalOdometry.pose.pose.orientation.x, _x.answer.globalOdometry.pose.pose.orientation.y, _x.answer.globalOdometry.pose.pose.orientation.z, _x.answer.globalOdometry.pose.pose.orientation.w))
      buff.write(_struct_36d.pack(*self.answer.globalOdometry.pose.covariance))
      _x = self
      buff.write(_struct_6d.pack(_x.answer.globalOdometry.twist.twist.linear.x, _x.answer.globalOdometry.twist.twist.linear.y, _x.answer.globalOdometry.twist.twist.linear.z, _x.answer.globalOdometry.twist.twist.angular.x, _x.answer.globalOdometry.twist.twist.angular.y, _x.answer.globalOdometry.twist.twist.angular.z))
      buff.write(_struct_36d.pack(*self.answer.globalOdometry.twist.covariance))
      _x = self
      buff.write(_struct_3I.pack(_x.answer.relativeOdometry.header.seq, _x.answer.relativeOdometry.header.stamp.secs, _x.answer.relativeOdometry.header.stamp.nsecs))
      _x = self.answer.relativeOdometry.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self.answer.relativeOdometry.child_frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_7d.pack(_x.answer.relativeOdometry.pose.pose.position.x, _x.answer.relativeOdometry.pose.pose.position.y, _x.answer.relativeOdometry.pose.pose.position.z, _x.answer.relativeOdometry.pose.pose.orientation.x, _x.answer.relativeOdometry.pose.pose.orientation.y, _x.answer.relativeOdometry.pose.pose.orientation.z, _x.answer.relativeOdometry.pose.pose.orientation.w))
      buff.write(_struct_36d.pack(*self.answer.relativeOdometry.pose.covariance))
      _x = self
      buff.write(_struct_6d.pack(_x.answer.relativeOdometry.twist.twist.linear.x, _x.answer.relativeOdometry.twist.twist.linear.y, _x.answer.relativeOdometry.twist.twist.linear.z, _x.answer.relativeOdometry.twist.twist.angular.x, _x.answer.relativeOdometry.twist.twist.angular.y, _x.answer.relativeOdometry.twist.twist.angular.z))
      buff.write(_struct_36d.pack(*self.answer.relativeOdometry.twist.covariance))
      buff.write(_struct_q.pack(self.answer.statusCode))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.answer is None:
        self.answer = my_odometry.msg.odom_answer()
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.answer.globalTF.transforms = []
      for i in range(0, length):
        val1 = geometry_msgs.msg.TransformStamped()
        _v11 = val1.header
        start = end
        end += 4
        (_v11.seq,) = _struct_I.unpack(str[start:end])
        _v12 = _v11.stamp
        _x = _v12
        start = end
        end += 8
        (_x.secs, _x.nsecs,) = _struct_2I.unpack(str[start:end])
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          _v11.frame_id = str[start:end].decode('utf-8')
        else:
          _v11.frame_id = str[start:end]
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1.child_frame_id = str[start:end].decode('utf-8')
        else:
          val1.child_frame_id = str[start:end]
        _v13 = val1.transform
        _v14 = _v13.translation
        _x = _v14
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _struct_3d.unpack(str[start:end])
        _v15 = _v13.rotation
        _x = _v15
        start = end
        end += 32
        (_x.x, _x.y, _x.z, _x.w,) = _struct_4d.unpack(str[start:end])
        self.answer.globalTF.transforms.append(val1)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.answer.relativeTF.transforms = []
      for i in range(0, length):
        val1 = geometry_msgs.msg.TransformStamped()
        _v16 = val1.header
        start = end
        end += 4
        (_v16.seq,) = _struct_I.unpack(str[start:end])
        _v17 = _v16.stamp
        _x = _v17
        start = end
        end += 8
        (_x.secs, _x.nsecs,) = _struct_2I.unpack(str[start:end])
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          _v16.frame_id = str[start:end].decode('utf-8')
        else:
          _v16.frame_id = str[start:end]
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1.child_frame_id = str[start:end].decode('utf-8')
        else:
          val1.child_frame_id = str[start:end]
        _v18 = val1.transform
        _v19 = _v18.translation
        _x = _v19
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _struct_3d.unpack(str[start:end])
        _v20 = _v18.rotation
        _x = _v20
        start = end
        end += 32
        (_x.x, _x.y, _x.z, _x.w,) = _struct_4d.unpack(str[start:end])
        self.answer.relativeTF.transforms.append(val1)
      _x = self
      start = end
      end += 12
      (_x.answer.globalOdometry.header.seq, _x.answer.globalOdometry.header.stamp.secs, _x.answer.globalOdometry.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.answer.globalOdometry.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.answer.globalOdometry.header.frame_id = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.answer.globalOdometry.child_frame_id = str[start:end].decode('utf-8')
      else:
        self.answer.globalOdometry.child_frame_id = str[start:end]
      _x = self
      start = end
      end += 56
      (_x.answer.globalOdometry.pose.pose.position.x, _x.answer.globalOdometry.pose.pose.position.y, _x.answer.globalOdometry.pose.pose.position.z, _x.answer.globalOdometry.pose.pose.orientation.x, _x.answer.globalOdometry.pose.pose.orientation.y, _x.answer.globalOdometry.pose.pose.orientation.z, _x.answer.globalOdometry.pose.pose.orientation.w,) = _struct_7d.unpack(str[start:end])
      start = end
      end += 288
      self.answer.globalOdometry.pose.covariance = _struct_36d.unpack(str[start:end])
      _x = self
      start = end
      end += 48
      (_x.answer.globalOdometry.twist.twist.linear.x, _x.answer.globalOdometry.twist.twist.linear.y, _x.answer.globalOdometry.twist.twist.linear.z, _x.answer.globalOdometry.twist.twist.angular.x, _x.answer.globalOdometry.twist.twist.angular.y, _x.answer.globalOdometry.twist.twist.angular.z,) = _struct_6d.unpack(str[start:end])
      start = end
      end += 288
      self.answer.globalOdometry.twist.covariance = _struct_36d.unpack(str[start:end])
      _x = self
      start = end
      end += 12
      (_x.answer.relativeOdometry.header.seq, _x.answer.relativeOdometry.header.stamp.secs, _x.answer.relativeOdometry.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.answer.relativeOdometry.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.answer.relativeOdometry.header.frame_id = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.answer.relativeOdometry.child_frame_id = str[start:end].decode('utf-8')
      else:
        self.answer.relativeOdometry.child_frame_id = str[start:end]
      _x = self
      start = end
      end += 56
      (_x.answer.relativeOdometry.pose.pose.position.x, _x.answer.relativeOdometry.pose.pose.position.y, _x.answer.relativeOdometry.pose.pose.position.z, _x.answer.relativeOdometry.pose.pose.orientation.x, _x.answer.relativeOdometry.pose.pose.orientation.y, _x.answer.relativeOdometry.pose.pose.orientation.z, _x.answer.relativeOdometry.pose.pose.orientation.w,) = _struct_7d.unpack(str[start:end])
      start = end
      end += 288
      self.answer.relativeOdometry.pose.covariance = _struct_36d.unpack(str[start:end])
      _x = self
      start = end
      end += 48
      (_x.answer.relativeOdometry.twist.twist.linear.x, _x.answer.relativeOdometry.twist.twist.linear.y, _x.answer.relativeOdometry.twist.twist.linear.z, _x.answer.relativeOdometry.twist.twist.angular.x, _x.answer.relativeOdometry.twist.twist.angular.y, _x.answer.relativeOdometry.twist.twist.angular.z,) = _struct_6d.unpack(str[start:end])
      start = end
      end += 288
      self.answer.relativeOdometry.twist.covariance = _struct_36d.unpack(str[start:end])
      start = end
      end += 8
      (self.answer.statusCode,) = _struct_q.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      length = len(self.answer.globalTF.transforms)
      buff.write(_struct_I.pack(length))
      for val1 in self.answer.globalTF.transforms:
        _v21 = val1.header
        buff.write(_struct_I.pack(_v21.seq))
        _v22 = _v21.stamp
        _x = _v22
        buff.write(_struct_2I.pack(_x.secs, _x.nsecs))
        _x = _v21.frame_id
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.pack('<I%ss'%length, length, _x))
        _x = val1.child_frame_id
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.pack('<I%ss'%length, length, _x))
        _v23 = val1.transform
        _v24 = _v23.translation
        _x = _v24
        buff.write(_struct_3d.pack(_x.x, _x.y, _x.z))
        _v25 = _v23.rotation
        _x = _v25
        buff.write(_struct_4d.pack(_x.x, _x.y, _x.z, _x.w))
      length = len(self.answer.relativeTF.transforms)
      buff.write(_struct_I.pack(length))
      for val1 in self.answer.relativeTF.transforms:
        _v26 = val1.header
        buff.write(_struct_I.pack(_v26.seq))
        _v27 = _v26.stamp
        _x = _v27
        buff.write(_struct_2I.pack(_x.secs, _x.nsecs))
        _x = _v26.frame_id
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.pack('<I%ss'%length, length, _x))
        _x = val1.child_frame_id
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.pack('<I%ss'%length, length, _x))
        _v28 = val1.transform
        _v29 = _v28.translation
        _x = _v29
        buff.write(_struct_3d.pack(_x.x, _x.y, _x.z))
        _v30 = _v28.rotation
        _x = _v30
        buff.write(_struct_4d.pack(_x.x, _x.y, _x.z, _x.w))
      _x = self
      buff.write(_struct_3I.pack(_x.answer.globalOdometry.header.seq, _x.answer.globalOdometry.header.stamp.secs, _x.answer.globalOdometry.header.stamp.nsecs))
      _x = self.answer.globalOdometry.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self.answer.globalOdometry.child_frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_7d.pack(_x.answer.globalOdometry.pose.pose.position.x, _x.answer.globalOdometry.pose.pose.position.y, _x.answer.globalOdometry.pose.pose.position.z, _x.answer.globalOdometry.pose.pose.orientation.x, _x.answer.globalOdometry.pose.pose.orientation.y, _x.answer.globalOdometry.pose.pose.orientation.z, _x.answer.globalOdometry.pose.pose.orientation.w))
      buff.write(self.answer.globalOdometry.pose.covariance.tostring())
      _x = self
      buff.write(_struct_6d.pack(_x.answer.globalOdometry.twist.twist.linear.x, _x.answer.globalOdometry.twist.twist.linear.y, _x.answer.globalOdometry.twist.twist.linear.z, _x.answer.globalOdometry.twist.twist.angular.x, _x.answer.globalOdometry.twist.twist.angular.y, _x.answer.globalOdometry.twist.twist.angular.z))
      buff.write(self.answer.globalOdometry.twist.covariance.tostring())
      _x = self
      buff.write(_struct_3I.pack(_x.answer.relativeOdometry.header.seq, _x.answer.relativeOdometry.header.stamp.secs, _x.answer.relativeOdometry.header.stamp.nsecs))
      _x = self.answer.relativeOdometry.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self.answer.relativeOdometry.child_frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_7d.pack(_x.answer.relativeOdometry.pose.pose.position.x, _x.answer.relativeOdometry.pose.pose.position.y, _x.answer.relativeOdometry.pose.pose.position.z, _x.answer.relativeOdometry.pose.pose.orientation.x, _x.answer.relativeOdometry.pose.pose.orientation.y, _x.answer.relativeOdometry.pose.pose.orientation.z, _x.answer.relativeOdometry.pose.pose.orientation.w))
      buff.write(self.answer.relativeOdometry.pose.covariance.tostring())
      _x = self
      buff.write(_struct_6d.pack(_x.answer.relativeOdometry.twist.twist.linear.x, _x.answer.relativeOdometry.twist.twist.linear.y, _x.answer.relativeOdometry.twist.twist.linear.z, _x.answer.relativeOdometry.twist.twist.angular.x, _x.answer.relativeOdometry.twist.twist.angular.y, _x.answer.relativeOdometry.twist.twist.angular.z))
      buff.write(self.answer.relativeOdometry.twist.covariance.tostring())
      buff.write(_struct_q.pack(self.answer.statusCode))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.answer is None:
        self.answer = my_odometry.msg.odom_answer()
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.answer.globalTF.transforms = []
      for i in range(0, length):
        val1 = geometry_msgs.msg.TransformStamped()
        _v31 = val1.header
        start = end
        end += 4
        (_v31.seq,) = _struct_I.unpack(str[start:end])
        _v32 = _v31.stamp
        _x = _v32
        start = end
        end += 8
        (_x.secs, _x.nsecs,) = _struct_2I.unpack(str[start:end])
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          _v31.frame_id = str[start:end].decode('utf-8')
        else:
          _v31.frame_id = str[start:end]
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1.child_frame_id = str[start:end].decode('utf-8')
        else:
          val1.child_frame_id = str[start:end]
        _v33 = val1.transform
        _v34 = _v33.translation
        _x = _v34
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _struct_3d.unpack(str[start:end])
        _v35 = _v33.rotation
        _x = _v35
        start = end
        end += 32
        (_x.x, _x.y, _x.z, _x.w,) = _struct_4d.unpack(str[start:end])
        self.answer.globalTF.transforms.append(val1)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.answer.relativeTF.transforms = []
      for i in range(0, length):
        val1 = geometry_msgs.msg.TransformStamped()
        _v36 = val1.header
        start = end
        end += 4
        (_v36.seq,) = _struct_I.unpack(str[start:end])
        _v37 = _v36.stamp
        _x = _v37
        start = end
        end += 8
        (_x.secs, _x.nsecs,) = _struct_2I.unpack(str[start:end])
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          _v36.frame_id = str[start:end].decode('utf-8')
        else:
          _v36.frame_id = str[start:end]
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1.child_frame_id = str[start:end].decode('utf-8')
        else:
          val1.child_frame_id = str[start:end]
        _v38 = val1.transform
        _v39 = _v38.translation
        _x = _v39
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _struct_3d.unpack(str[start:end])
        _v40 = _v38.rotation
        _x = _v40
        start = end
        end += 32
        (_x.x, _x.y, _x.z, _x.w,) = _struct_4d.unpack(str[start:end])
        self.answer.relativeTF.transforms.append(val1)
      _x = self
      start = end
      end += 12
      (_x.answer.globalOdometry.header.seq, _x.answer.globalOdometry.header.stamp.secs, _x.answer.globalOdometry.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.answer.globalOdometry.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.answer.globalOdometry.header.frame_id = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.answer.globalOdometry.child_frame_id = str[start:end].decode('utf-8')
      else:
        self.answer.globalOdometry.child_frame_id = str[start:end]
      _x = self
      start = end
      end += 56
      (_x.answer.globalOdometry.pose.pose.position.x, _x.answer.globalOdometry.pose.pose.position.y, _x.answer.globalOdometry.pose.pose.position.z, _x.answer.globalOdometry.pose.pose.orientation.x, _x.answer.globalOdometry.pose.pose.orientation.y, _x.answer.globalOdometry.pose.pose.orientation.z, _x.answer.globalOdometry.pose.pose.orientation.w,) = _struct_7d.unpack(str[start:end])
      start = end
      end += 288
      self.answer.globalOdometry.pose.covariance = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=36)
      _x = self
      start = end
      end += 48
      (_x.answer.globalOdometry.twist.twist.linear.x, _x.answer.globalOdometry.twist.twist.linear.y, _x.answer.globalOdometry.twist.twist.linear.z, _x.answer.globalOdometry.twist.twist.angular.x, _x.answer.globalOdometry.twist.twist.angular.y, _x.answer.globalOdometry.twist.twist.angular.z,) = _struct_6d.unpack(str[start:end])
      start = end
      end += 288
      self.answer.globalOdometry.twist.covariance = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=36)
      _x = self
      start = end
      end += 12
      (_x.answer.relativeOdometry.header.seq, _x.answer.relativeOdometry.header.stamp.secs, _x.answer.relativeOdometry.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.answer.relativeOdometry.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.answer.relativeOdometry.header.frame_id = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.answer.relativeOdometry.child_frame_id = str[start:end].decode('utf-8')
      else:
        self.answer.relativeOdometry.child_frame_id = str[start:end]
      _x = self
      start = end
      end += 56
      (_x.answer.relativeOdometry.pose.pose.position.x, _x.answer.relativeOdometry.pose.pose.position.y, _x.answer.relativeOdometry.pose.pose.position.z, _x.answer.relativeOdometry.pose.pose.orientation.x, _x.answer.relativeOdometry.pose.pose.orientation.y, _x.answer.relativeOdometry.pose.pose.orientation.z, _x.answer.relativeOdometry.pose.pose.orientation.w,) = _struct_7d.unpack(str[start:end])
      start = end
      end += 288
      self.answer.relativeOdometry.pose.covariance = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=36)
      _x = self
      start = end
      end += 48
      (_x.answer.relativeOdometry.twist.twist.linear.x, _x.answer.relativeOdometry.twist.twist.linear.y, _x.answer.relativeOdometry.twist.twist.linear.z, _x.answer.relativeOdometry.twist.twist.angular.x, _x.answer.relativeOdometry.twist.twist.angular.y, _x.answer.relativeOdometry.twist.twist.angular.z,) = _struct_6d.unpack(str[start:end])
      start = end
      end += 288
      self.answer.relativeOdometry.twist.covariance = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=36)
      start = end
      end += 8
      (self.answer.statusCode,) = _struct_q.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_7d = struct.Struct("<7d")
_struct_6d = struct.Struct("<6d")
_struct_36d = struct.Struct("<36d")
_struct_q = struct.Struct("<q")
_struct_3I = struct.Struct("<3I")
_struct_4d = struct.Struct("<4d")
_struct_2I = struct.Struct("<2I")
_struct_3d = struct.Struct("<3d")
class emptyRequest(object):
  _type          = 'my_odometry/emptyRequest'
  _md5sum = '0fd54208cf1a13d4feef7d72c284d8d1'
  _request_class  = emptyRequestRequest
  _response_class = emptyRequestResponse

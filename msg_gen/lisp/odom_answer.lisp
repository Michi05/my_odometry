; Auto-generated. Do not edit!


(cl:in-package my_odometry-msg)


;//! \htmlinclude odom_answer.msg.html

(cl:defclass <odom_answer> (roslisp-msg-protocol:ros-message)
  ((globalTF
    :reader globalTF
    :initarg :globalTF
    :type tf2_msgs-msg:TFMessage
    :initform (cl:make-instance 'tf2_msgs-msg:TFMessage))
   (relativeTF
    :reader relativeTF
    :initarg :relativeTF
    :type tf2_msgs-msg:TFMessage
    :initform (cl:make-instance 'tf2_msgs-msg:TFMessage))
   (globalOdometry
    :reader globalOdometry
    :initarg :globalOdometry
    :type nav_msgs-msg:Odometry
    :initform (cl:make-instance 'nav_msgs-msg:Odometry))
   (relativeOdometry
    :reader relativeOdometry
    :initarg :relativeOdometry
    :type nav_msgs-msg:Odometry
    :initform (cl:make-instance 'nav_msgs-msg:Odometry))
   (statusCode
    :reader statusCode
    :initarg :statusCode
    :type cl:integer
    :initform 0))
)

(cl:defclass odom_answer (<odom_answer>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <odom_answer>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'odom_answer)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name my_odometry-msg:<odom_answer> is deprecated: use my_odometry-msg:odom_answer instead.")))

(cl:ensure-generic-function 'globalTF-val :lambda-list '(m))
(cl:defmethod globalTF-val ((m <odom_answer>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader my_odometry-msg:globalTF-val is deprecated.  Use my_odometry-msg:globalTF instead.")
  (globalTF m))

(cl:ensure-generic-function 'relativeTF-val :lambda-list '(m))
(cl:defmethod relativeTF-val ((m <odom_answer>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader my_odometry-msg:relativeTF-val is deprecated.  Use my_odometry-msg:relativeTF instead.")
  (relativeTF m))

(cl:ensure-generic-function 'globalOdometry-val :lambda-list '(m))
(cl:defmethod globalOdometry-val ((m <odom_answer>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader my_odometry-msg:globalOdometry-val is deprecated.  Use my_odometry-msg:globalOdometry instead.")
  (globalOdometry m))

(cl:ensure-generic-function 'relativeOdometry-val :lambda-list '(m))
(cl:defmethod relativeOdometry-val ((m <odom_answer>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader my_odometry-msg:relativeOdometry-val is deprecated.  Use my_odometry-msg:relativeOdometry instead.")
  (relativeOdometry m))

(cl:ensure-generic-function 'statusCode-val :lambda-list '(m))
(cl:defmethod statusCode-val ((m <odom_answer>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader my_odometry-msg:statusCode-val is deprecated.  Use my_odometry-msg:statusCode instead.")
  (statusCode m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <odom_answer>) ostream)
  "Serializes a message object of type '<odom_answer>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'globalTF) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'relativeTF) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'globalOdometry) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'relativeOdometry) ostream)
  (cl:let* ((signed (cl:slot-value msg 'statusCode)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <odom_answer>) istream)
  "Deserializes a message object of type '<odom_answer>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'globalTF) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'relativeTF) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'globalOdometry) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'relativeOdometry) istream)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'statusCode) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<odom_answer>)))
  "Returns string type for a message object of type '<odom_answer>"
  "my_odometry/odom_answer")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'odom_answer)))
  "Returns string type for a message object of type 'odom_answer"
  "my_odometry/odom_answer")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<odom_answer>)))
  "Returns md5sum for a message object of type '<odom_answer>"
  "924973a8bd033ee323eab0377af1373a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'odom_answer)))
  "Returns md5sum for a message object of type 'odom_answer"
  "924973a8bd033ee323eab0377af1373a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<odom_answer>)))
  "Returns full string definition for message of type '<odom_answer>"
  (cl:format cl:nil "## Common answer message for the odometry package~%##the services are suppossed to answer with this~%##info everytime.~%~%#   Global/Relative transform/odometry messages:~%# Transform frame from tf2 package~%tf2_msgs/TFMessage globalTF~%tf2_msgs/TFMessage relativeTF~%# System's odometry messages~%nav_msgs/Odometry globalOdometry~%nav_msgs/Odometry relativeOdometry~%~%# Code number for the status of the whole algorithm~%int64 statusCode~%~%~%================================================================================~%MSG: tf2_msgs/TFMessage~%geometry_msgs/TransformStamped[] transforms~%~%================================================================================~%MSG: geometry_msgs/TransformStamped~%# This expresses a transform from coordinate frame header.frame_id~%# to the coordinate frame child_frame_id~%#~%# This message is mostly used by the ~%# <a href=\"http://www.ros.org/wiki/tf\">tf</a> package. ~%# See it's documentation for more information.~%~%Header header~%string child_frame_id # the frame id of the child frame~%Transform transform~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Transform~%# This represents the transform between two coordinate frames in free space.~%~%Vector3 translation~%Quaternion rotation~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: nav_msgs/Odometry~%# This represents an estimate of a position and velocity in free space.  ~%# The pose in this message should be specified in the coordinate frame given by header.frame_id.~%# The twist in this message should be specified in the coordinate frame given by the child_frame_id~%Header header~%string child_frame_id~%geometry_msgs/PoseWithCovariance pose~%geometry_msgs/TwistWithCovariance twist~%~%================================================================================~%MSG: geometry_msgs/PoseWithCovariance~%# This represents a pose in free space with uncertainty.~%~%Pose pose~%~%# Row-major representation of the 6x6 covariance matrix~%# The orientation parameters use a fixed-axis representation.~%# In order, the parameters are:~%# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)~%float64[36] covariance~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/TwistWithCovariance~%# This expresses velocity in free space with uncertianty.~%~%Twist twist~%~%# Row-major representation of the 6x6 covariance matrix~%# The orientation parameters use a fixed-axis representation.~%# In order, the parameters are:~%# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)~%float64[36] covariance~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into it's linear and angular parts. ~%Vector3  linear~%Vector3  angular~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'odom_answer)))
  "Returns full string definition for message of type 'odom_answer"
  (cl:format cl:nil "## Common answer message for the odometry package~%##the services are suppossed to answer with this~%##info everytime.~%~%#   Global/Relative transform/odometry messages:~%# Transform frame from tf2 package~%tf2_msgs/TFMessage globalTF~%tf2_msgs/TFMessage relativeTF~%# System's odometry messages~%nav_msgs/Odometry globalOdometry~%nav_msgs/Odometry relativeOdometry~%~%# Code number for the status of the whole algorithm~%int64 statusCode~%~%~%================================================================================~%MSG: tf2_msgs/TFMessage~%geometry_msgs/TransformStamped[] transforms~%~%================================================================================~%MSG: geometry_msgs/TransformStamped~%# This expresses a transform from coordinate frame header.frame_id~%# to the coordinate frame child_frame_id~%#~%# This message is mostly used by the ~%# <a href=\"http://www.ros.org/wiki/tf\">tf</a> package. ~%# See it's documentation for more information.~%~%Header header~%string child_frame_id # the frame id of the child frame~%Transform transform~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Transform~%# This represents the transform between two coordinate frames in free space.~%~%Vector3 translation~%Quaternion rotation~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: nav_msgs/Odometry~%# This represents an estimate of a position and velocity in free space.  ~%# The pose in this message should be specified in the coordinate frame given by header.frame_id.~%# The twist in this message should be specified in the coordinate frame given by the child_frame_id~%Header header~%string child_frame_id~%geometry_msgs/PoseWithCovariance pose~%geometry_msgs/TwistWithCovariance twist~%~%================================================================================~%MSG: geometry_msgs/PoseWithCovariance~%# This represents a pose in free space with uncertainty.~%~%Pose pose~%~%# Row-major representation of the 6x6 covariance matrix~%# The orientation parameters use a fixed-axis representation.~%# In order, the parameters are:~%# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)~%float64[36] covariance~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/TwistWithCovariance~%# This expresses velocity in free space with uncertianty.~%~%Twist twist~%~%# Row-major representation of the 6x6 covariance matrix~%# The orientation parameters use a fixed-axis representation.~%# In order, the parameters are:~%# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)~%float64[36] covariance~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into it's linear and angular parts. ~%Vector3  linear~%Vector3  angular~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <odom_answer>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'globalTF))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'relativeTF))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'globalOdometry))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'relativeOdometry))
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <odom_answer>))
  "Converts a ROS message object to a list"
  (cl:list 'odom_answer
    (cl:cons ':globalTF (globalTF msg))
    (cl:cons ':relativeTF (relativeTF msg))
    (cl:cons ':globalOdometry (globalOdometry msg))
    (cl:cons ':relativeOdometry (relativeOdometry msg))
    (cl:cons ':statusCode (statusCode msg))
))

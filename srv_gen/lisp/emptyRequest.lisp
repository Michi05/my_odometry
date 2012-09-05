; Auto-generated. Do not edit!


(cl:in-package my_odometry-srv)


;//! \htmlinclude emptyRequest-request.msg.html

(cl:defclass <emptyRequest-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass emptyRequest-request (<emptyRequest-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <emptyRequest-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'emptyRequest-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name my_odometry-srv:<emptyRequest-request> is deprecated: use my_odometry-srv:emptyRequest-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <emptyRequest-request>) ostream)
  "Serializes a message object of type '<emptyRequest-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <emptyRequest-request>) istream)
  "Deserializes a message object of type '<emptyRequest-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<emptyRequest-request>)))
  "Returns string type for a service object of type '<emptyRequest-request>"
  "my_odometry/emptyRequestRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'emptyRequest-request)))
  "Returns string type for a service object of type 'emptyRequest-request"
  "my_odometry/emptyRequestRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<emptyRequest-request>)))
  "Returns md5sum for a message object of type '<emptyRequest-request>"
  "0fd54208cf1a13d4feef7d72c284d8d1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'emptyRequest-request)))
  "Returns md5sum for a message object of type 'emptyRequest-request"
  "0fd54208cf1a13d4feef7d72c284d8d1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<emptyRequest-request>)))
  "Returns full string definition for message of type '<emptyRequest-request>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'emptyRequest-request)))
  "Returns full string definition for message of type 'emptyRequest-request"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <emptyRequest-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <emptyRequest-request>))
  "Converts a ROS message object to a list"
  (cl:list 'emptyRequest-request
))
;//! \htmlinclude emptyRequest-response.msg.html

(cl:defclass <emptyRequest-response> (roslisp-msg-protocol:ros-message)
  ((answer
    :reader answer
    :initarg :answer
    :type my_odometry-msg:odom_answer
    :initform (cl:make-instance 'my_odometry-msg:odom_answer)))
)

(cl:defclass emptyRequest-response (<emptyRequest-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <emptyRequest-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'emptyRequest-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name my_odometry-srv:<emptyRequest-response> is deprecated: use my_odometry-srv:emptyRequest-response instead.")))

(cl:ensure-generic-function 'answer-val :lambda-list '(m))
(cl:defmethod answer-val ((m <emptyRequest-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader my_odometry-srv:answer-val is deprecated.  Use my_odometry-srv:answer instead.")
  (answer m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <emptyRequest-response>) ostream)
  "Serializes a message object of type '<emptyRequest-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'answer) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <emptyRequest-response>) istream)
  "Deserializes a message object of type '<emptyRequest-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'answer) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<emptyRequest-response>)))
  "Returns string type for a service object of type '<emptyRequest-response>"
  "my_odometry/emptyRequestResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'emptyRequest-response)))
  "Returns string type for a service object of type 'emptyRequest-response"
  "my_odometry/emptyRequestResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<emptyRequest-response>)))
  "Returns md5sum for a message object of type '<emptyRequest-response>"
  "0fd54208cf1a13d4feef7d72c284d8d1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'emptyRequest-response)))
  "Returns md5sum for a message object of type 'emptyRequest-response"
  "0fd54208cf1a13d4feef7d72c284d8d1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<emptyRequest-response>)))
  "Returns full string definition for message of type '<emptyRequest-response>"
  (cl:format cl:nil "my_odometry/odom_answer answer~%~%~%~%================================================================================~%MSG: my_odometry/odom_answer~%## Common answer message for the odometry package~%##the services are suppossed to answer with this~%##info everytime.~%~%#   Global/Relative transform/odometry messages:~%# Transform frame from tf2 package~%tf2_msgs/TFMessage globalTF~%tf2_msgs/TFMessage relativeTF~%# System's odometry messages~%nav_msgs/Odometry globalOdometry~%nav_msgs/Odometry relativeOdometry~%~%# Code number for the status of the whole algorithm~%int64 statusCode~%~%~%================================================================================~%MSG: tf2_msgs/TFMessage~%geometry_msgs/TransformStamped[] transforms~%~%================================================================================~%MSG: geometry_msgs/TransformStamped~%# This expresses a transform from coordinate frame header.frame_id~%# to the coordinate frame child_frame_id~%#~%# This message is mostly used by the ~%# <a href=\"http://www.ros.org/wiki/tf\">tf</a> package. ~%# See it's documentation for more information.~%~%Header header~%string child_frame_id # the frame id of the child frame~%Transform transform~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Transform~%# This represents the transform between two coordinate frames in free space.~%~%Vector3 translation~%Quaternion rotation~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: nav_msgs/Odometry~%# This represents an estimate of a position and velocity in free space.  ~%# The pose in this message should be specified in the coordinate frame given by header.frame_id.~%# The twist in this message should be specified in the coordinate frame given by the child_frame_id~%Header header~%string child_frame_id~%geometry_msgs/PoseWithCovariance pose~%geometry_msgs/TwistWithCovariance twist~%~%================================================================================~%MSG: geometry_msgs/PoseWithCovariance~%# This represents a pose in free space with uncertainty.~%~%Pose pose~%~%# Row-major representation of the 6x6 covariance matrix~%# The orientation parameters use a fixed-axis representation.~%# In order, the parameters are:~%# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)~%float64[36] covariance~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/TwistWithCovariance~%# This expresses velocity in free space with uncertianty.~%~%Twist twist~%~%# Row-major representation of the 6x6 covariance matrix~%# The orientation parameters use a fixed-axis representation.~%# In order, the parameters are:~%# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)~%float64[36] covariance~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into it's linear and angular parts. ~%Vector3  linear~%Vector3  angular~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'emptyRequest-response)))
  "Returns full string definition for message of type 'emptyRequest-response"
  (cl:format cl:nil "my_odometry/odom_answer answer~%~%~%~%================================================================================~%MSG: my_odometry/odom_answer~%## Common answer message for the odometry package~%##the services are suppossed to answer with this~%##info everytime.~%~%#   Global/Relative transform/odometry messages:~%# Transform frame from tf2 package~%tf2_msgs/TFMessage globalTF~%tf2_msgs/TFMessage relativeTF~%# System's odometry messages~%nav_msgs/Odometry globalOdometry~%nav_msgs/Odometry relativeOdometry~%~%# Code number for the status of the whole algorithm~%int64 statusCode~%~%~%================================================================================~%MSG: tf2_msgs/TFMessage~%geometry_msgs/TransformStamped[] transforms~%~%================================================================================~%MSG: geometry_msgs/TransformStamped~%# This expresses a transform from coordinate frame header.frame_id~%# to the coordinate frame child_frame_id~%#~%# This message is mostly used by the ~%# <a href=\"http://www.ros.org/wiki/tf\">tf</a> package. ~%# See it's documentation for more information.~%~%Header header~%string child_frame_id # the frame id of the child frame~%Transform transform~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Transform~%# This represents the transform between two coordinate frames in free space.~%~%Vector3 translation~%Quaternion rotation~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: nav_msgs/Odometry~%# This represents an estimate of a position and velocity in free space.  ~%# The pose in this message should be specified in the coordinate frame given by header.frame_id.~%# The twist in this message should be specified in the coordinate frame given by the child_frame_id~%Header header~%string child_frame_id~%geometry_msgs/PoseWithCovariance pose~%geometry_msgs/TwistWithCovariance twist~%~%================================================================================~%MSG: geometry_msgs/PoseWithCovariance~%# This represents a pose in free space with uncertainty.~%~%Pose pose~%~%# Row-major representation of the 6x6 covariance matrix~%# The orientation parameters use a fixed-axis representation.~%# In order, the parameters are:~%# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)~%float64[36] covariance~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/TwistWithCovariance~%# This expresses velocity in free space with uncertianty.~%~%Twist twist~%~%# Row-major representation of the 6x6 covariance matrix~%# The orientation parameters use a fixed-axis representation.~%# In order, the parameters are:~%# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)~%float64[36] covariance~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into it's linear and angular parts. ~%Vector3  linear~%Vector3  angular~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <emptyRequest-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'answer))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <emptyRequest-response>))
  "Converts a ROS message object to a list"
  (cl:list 'emptyRequest-response
    (cl:cons ':answer (answer msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'emptyRequest)))
  'emptyRequest-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'emptyRequest)))
  'emptyRequest-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'emptyRequest)))
  "Returns string type for a service object of type '<emptyRequest>"
  "my_odometry/emptyRequest")
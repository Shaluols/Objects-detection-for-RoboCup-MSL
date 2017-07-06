; Auto-generated. Do not edit!


(cl:in-package nubot_common-msg)


;//! \htmlinclude object_info.msg.html

(cl:defclass <object_info> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (ball_pos
    :reader ball_pos
    :initarg :ball_pos
    :type (cl:vector geometry_msgs-msg:Point)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Point :initial-element (cl:make-instance 'geometry_msgs-msg:Point)))
   (obs_pos
    :reader obs_pos
    :initarg :obs_pos
    :type (cl:vector geometry_msgs-msg:Point)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Point :initial-element (cl:make-instance 'geometry_msgs-msg:Point)))
   (ball_know
    :reader ball_know
    :initarg :ball_know
    :type cl:boolean
    :initform cl:nil)
   (obs_know
    :reader obs_know
    :initarg :obs_know
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass object_info (<object_info>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <object_info>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'object_info)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name nubot_common-msg:<object_info> is deprecated: use nubot_common-msg:object_info instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <object_info>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader nubot_common-msg:header-val is deprecated.  Use nubot_common-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'ball_pos-val :lambda-list '(m))
(cl:defmethod ball_pos-val ((m <object_info>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader nubot_common-msg:ball_pos-val is deprecated.  Use nubot_common-msg:ball_pos instead.")
  (ball_pos m))

(cl:ensure-generic-function 'obs_pos-val :lambda-list '(m))
(cl:defmethod obs_pos-val ((m <object_info>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader nubot_common-msg:obs_pos-val is deprecated.  Use nubot_common-msg:obs_pos instead.")
  (obs_pos m))

(cl:ensure-generic-function 'ball_know-val :lambda-list '(m))
(cl:defmethod ball_know-val ((m <object_info>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader nubot_common-msg:ball_know-val is deprecated.  Use nubot_common-msg:ball_know instead.")
  (ball_know m))

(cl:ensure-generic-function 'obs_know-val :lambda-list '(m))
(cl:defmethod obs_know-val ((m <object_info>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader nubot_common-msg:obs_know-val is deprecated.  Use nubot_common-msg:obs_know instead.")
  (obs_know m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <object_info>) ostream)
  "Serializes a message object of type '<object_info>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'ball_pos))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'ball_pos))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'obs_pos))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'obs_pos))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'ball_know) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'obs_know) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <object_info>) istream)
  "Deserializes a message object of type '<object_info>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'ball_pos) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'ball_pos)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Point))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'obs_pos) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'obs_pos)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Point))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
    (cl:setf (cl:slot-value msg 'ball_know) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'obs_know) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<object_info>)))
  "Returns string type for a message object of type '<object_info>"
  "nubot_common/object_info")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'object_info)))
  "Returns string type for a message object of type 'object_info"
  "nubot_common/object_info")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<object_info>)))
  "Returns md5sum for a message object of type '<object_info>"
  "799347d7afb7b966d6574b6b5aa1c5a4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'object_info)))
  "Returns md5sum for a message object of type 'object_info"
  "799347d7afb7b966d6574b6b5aa1c5a4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<object_info>)))
  "Returns full string definition for message of type '<object_info>"
  (cl:format cl:nil "Header header~%geometry_msgs/Point[] ball_pos~%geometry_msgs/Point[] obs_pos~%bool ball_know~%bool obs_know~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'object_info)))
  "Returns full string definition for message of type 'object_info"
  (cl:format cl:nil "Header header~%geometry_msgs/Point[] ball_pos~%geometry_msgs/Point[] obs_pos~%bool ball_know~%bool obs_know~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <object_info>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'ball_pos) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'obs_pos) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <object_info>))
  "Converts a ROS message object to a list"
  (cl:list 'object_info
    (cl:cons ':header (header msg))
    (cl:cons ':ball_pos (ball_pos msg))
    (cl:cons ':obs_pos (obs_pos msg))
    (cl:cons ':ball_know (ball_know msg))
    (cl:cons ':obs_know (obs_know msg))
))

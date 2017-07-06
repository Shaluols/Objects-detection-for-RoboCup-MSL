; Auto-generated. Do not edit!


(cl:in-package nubot_common-msg)


;//! \htmlinclude ObstaclesInfo3d.msg.html

(cl:defclass <ObstaclesInfo3d> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (pos
    :reader pos
    :initarg :pos
    :type (cl:vector nubot_common-msg:Point3d)
   :initform (cl:make-array 0 :element-type 'nubot_common-msg:Point3d :initial-element (cl:make-instance 'nubot_common-msg:Point3d)))
   (pos_min
    :reader pos_min
    :initarg :pos_min
    :type nubot_common-msg:Point3d
    :initform (cl:make-instance 'nubot_common-msg:Point3d))
   (pos_known_3d
    :reader pos_known_3d
    :initarg :pos_known_3d
    :type cl:boolean
    :initform cl:nil)
   (pos_known_2d
    :reader pos_known_2d
    :initarg :pos_known_2d
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass ObstaclesInfo3d (<ObstaclesInfo3d>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ObstaclesInfo3d>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ObstaclesInfo3d)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name nubot_common-msg:<ObstaclesInfo3d> is deprecated: use nubot_common-msg:ObstaclesInfo3d instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <ObstaclesInfo3d>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader nubot_common-msg:header-val is deprecated.  Use nubot_common-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'pos-val :lambda-list '(m))
(cl:defmethod pos-val ((m <ObstaclesInfo3d>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader nubot_common-msg:pos-val is deprecated.  Use nubot_common-msg:pos instead.")
  (pos m))

(cl:ensure-generic-function 'pos_min-val :lambda-list '(m))
(cl:defmethod pos_min-val ((m <ObstaclesInfo3d>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader nubot_common-msg:pos_min-val is deprecated.  Use nubot_common-msg:pos_min instead.")
  (pos_min m))

(cl:ensure-generic-function 'pos_known_3d-val :lambda-list '(m))
(cl:defmethod pos_known_3d-val ((m <ObstaclesInfo3d>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader nubot_common-msg:pos_known_3d-val is deprecated.  Use nubot_common-msg:pos_known_3d instead.")
  (pos_known_3d m))

(cl:ensure-generic-function 'pos_known_2d-val :lambda-list '(m))
(cl:defmethod pos_known_2d-val ((m <ObstaclesInfo3d>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader nubot_common-msg:pos_known_2d-val is deprecated.  Use nubot_common-msg:pos_known_2d instead.")
  (pos_known_2d m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ObstaclesInfo3d>) ostream)
  "Serializes a message object of type '<ObstaclesInfo3d>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'pos))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'pos))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pos_min) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'pos_known_3d) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'pos_known_2d) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ObstaclesInfo3d>) istream)
  "Deserializes a message object of type '<ObstaclesInfo3d>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'pos) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'pos)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'nubot_common-msg:Point3d))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pos_min) istream)
    (cl:setf (cl:slot-value msg 'pos_known_3d) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'pos_known_2d) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ObstaclesInfo3d>)))
  "Returns string type for a message object of type '<ObstaclesInfo3d>"
  "nubot_common/ObstaclesInfo3d")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ObstaclesInfo3d)))
  "Returns string type for a message object of type 'ObstaclesInfo3d"
  "nubot_common/ObstaclesInfo3d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ObstaclesInfo3d>)))
  "Returns md5sum for a message object of type '<ObstaclesInfo3d>"
  "60241ee2c1942a7ff4999dfb0ac383c8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ObstaclesInfo3d)))
  "Returns md5sum for a message object of type 'ObstaclesInfo3d"
  "60241ee2c1942a7ff4999dfb0ac383c8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ObstaclesInfo3d>)))
  "Returns full string definition for message of type '<ObstaclesInfo3d>"
  (cl:format cl:nil "Header header~%Point3d[]   pos~%Point3d     pos_min~%~%bool      pos_known_3d~%bool      pos_known_2d~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: nubot_common/Point3d~%float32 x~%float32 y~%float32 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ObstaclesInfo3d)))
  "Returns full string definition for message of type 'ObstaclesInfo3d"
  (cl:format cl:nil "Header header~%Point3d[]   pos~%Point3d     pos_min~%~%bool      pos_known_3d~%bool      pos_known_2d~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: nubot_common/Point3d~%float32 x~%float32 y~%float32 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ObstaclesInfo3d>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'pos) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pos_min))
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ObstaclesInfo3d>))
  "Converts a ROS message object to a list"
  (cl:list 'ObstaclesInfo3d
    (cl:cons ':header (header msg))
    (cl:cons ':pos (pos msg))
    (cl:cons ':pos_min (pos_min msg))
    (cl:cons ':pos_known_3d (pos_known_3d msg))
    (cl:cons ':pos_known_2d (pos_known_2d msg))
))

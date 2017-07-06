; Auto-generated. Do not edit!


(cl:in-package nubot_common-msg)


;//! \htmlinclude VelCmd.msg.html

(cl:defclass <VelCmd> (roslisp-msg-protocol:ros-message)
  ((target
    :reader target
    :initarg :target
    :type nubot_common-msg:Point2d
    :initform (cl:make-instance 'nubot_common-msg:Point2d))
   (target_ori
    :reader target_ori
    :initarg :target_ori
    :type cl:float
    :initform 0.0)
   (target_vel
    :reader target_vel
    :initarg :target_vel
    :type nubot_common-msg:Point2d
    :initform (cl:make-instance 'nubot_common-msg:Point2d))
   (maxvel
    :reader maxvel
    :initarg :maxvel
    :type cl:float
    :initform 0.0)
   (maxw
    :reader maxw
    :initarg :maxw
    :type cl:float
    :initform 0.0)
   (robot_pos
    :reader robot_pos
    :initarg :robot_pos
    :type nubot_common-msg:Point2d
    :initform (cl:make-instance 'nubot_common-msg:Point2d))
   (robot_ori
    :reader robot_ori
    :initarg :robot_ori
    :type cl:float
    :initform 0.0)
   (move_action
    :reader move_action
    :initarg :move_action
    :type cl:integer
    :initform 0)
   (rotate_acton
    :reader rotate_acton
    :initarg :rotate_acton
    :type cl:integer
    :initform 0))
)

(cl:defclass VelCmd (<VelCmd>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <VelCmd>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'VelCmd)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name nubot_common-msg:<VelCmd> is deprecated: use nubot_common-msg:VelCmd instead.")))

(cl:ensure-generic-function 'target-val :lambda-list '(m))
(cl:defmethod target-val ((m <VelCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader nubot_common-msg:target-val is deprecated.  Use nubot_common-msg:target instead.")
  (target m))

(cl:ensure-generic-function 'target_ori-val :lambda-list '(m))
(cl:defmethod target_ori-val ((m <VelCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader nubot_common-msg:target_ori-val is deprecated.  Use nubot_common-msg:target_ori instead.")
  (target_ori m))

(cl:ensure-generic-function 'target_vel-val :lambda-list '(m))
(cl:defmethod target_vel-val ((m <VelCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader nubot_common-msg:target_vel-val is deprecated.  Use nubot_common-msg:target_vel instead.")
  (target_vel m))

(cl:ensure-generic-function 'maxvel-val :lambda-list '(m))
(cl:defmethod maxvel-val ((m <VelCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader nubot_common-msg:maxvel-val is deprecated.  Use nubot_common-msg:maxvel instead.")
  (maxvel m))

(cl:ensure-generic-function 'maxw-val :lambda-list '(m))
(cl:defmethod maxw-val ((m <VelCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader nubot_common-msg:maxw-val is deprecated.  Use nubot_common-msg:maxw instead.")
  (maxw m))

(cl:ensure-generic-function 'robot_pos-val :lambda-list '(m))
(cl:defmethod robot_pos-val ((m <VelCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader nubot_common-msg:robot_pos-val is deprecated.  Use nubot_common-msg:robot_pos instead.")
  (robot_pos m))

(cl:ensure-generic-function 'robot_ori-val :lambda-list '(m))
(cl:defmethod robot_ori-val ((m <VelCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader nubot_common-msg:robot_ori-val is deprecated.  Use nubot_common-msg:robot_ori instead.")
  (robot_ori m))

(cl:ensure-generic-function 'move_action-val :lambda-list '(m))
(cl:defmethod move_action-val ((m <VelCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader nubot_common-msg:move_action-val is deprecated.  Use nubot_common-msg:move_action instead.")
  (move_action m))

(cl:ensure-generic-function 'rotate_acton-val :lambda-list '(m))
(cl:defmethod rotate_acton-val ((m <VelCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader nubot_common-msg:rotate_acton-val is deprecated.  Use nubot_common-msg:rotate_acton instead.")
  (rotate_acton m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <VelCmd>) ostream)
  "Serializes a message object of type '<VelCmd>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'target) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'target_ori))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'target_vel) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'maxvel))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'maxw))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'robot_pos) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'robot_ori))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'move_action)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'rotate_acton)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <VelCmd>) istream)
  "Deserializes a message object of type '<VelCmd>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'target) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'target_ori) (roslisp-utils:decode-single-float-bits bits)))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'target_vel) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'maxvel) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'maxw) (roslisp-utils:decode-single-float-bits bits)))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'robot_pos) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'robot_ori) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'move_action)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'rotate_acton)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<VelCmd>)))
  "Returns string type for a message object of type '<VelCmd>"
  "nubot_common/VelCmd")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'VelCmd)))
  "Returns string type for a message object of type 'VelCmd"
  "nubot_common/VelCmd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<VelCmd>)))
  "Returns md5sum for a message object of type '<VelCmd>"
  "24f8f6a4f2243802ffe8d3de8b91cf6b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'VelCmd)))
  "Returns md5sum for a message object of type 'VelCmd"
  "24f8f6a4f2243802ffe8d3de8b91cf6b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<VelCmd>)))
  "Returns full string definition for message of type '<VelCmd>"
  (cl:format cl:nil "Point2d target~%float32 target_ori~%Point2d target_vel~%float32 maxvel~%float32 maxw~%Point2d robot_pos~%float32   robot_ori~%char    move_action~%char    rotate_acton~%~%================================================================================~%MSG: nubot_common/Point2d~%float32 x~%float32 y~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'VelCmd)))
  "Returns full string definition for message of type 'VelCmd"
  (cl:format cl:nil "Point2d target~%float32 target_ori~%Point2d target_vel~%float32 maxvel~%float32 maxw~%Point2d robot_pos~%float32   robot_ori~%char    move_action~%char    rotate_acton~%~%================================================================================~%MSG: nubot_common/Point2d~%float32 x~%float32 y~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <VelCmd>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'target))
     4
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'target_vel))
     4
     4
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'robot_pos))
     4
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <VelCmd>))
  "Converts a ROS message object to a list"
  (cl:list 'VelCmd
    (cl:cons ':target (target msg))
    (cl:cons ':target_ori (target_ori msg))
    (cl:cons ':target_vel (target_vel msg))
    (cl:cons ':maxvel (maxvel msg))
    (cl:cons ':maxw (maxw msg))
    (cl:cons ':robot_pos (robot_pos msg))
    (cl:cons ':robot_ori (robot_ori msg))
    (cl:cons ':move_action (move_action msg))
    (cl:cons ':rotate_acton (rotate_acton msg))
))

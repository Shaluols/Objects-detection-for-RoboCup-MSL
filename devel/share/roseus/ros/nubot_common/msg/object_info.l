;; Auto-generated. Do not edit!


(when (boundp 'nubot_common::object_info)
  (if (not (find-package "NUBOT_COMMON"))
    (make-package "NUBOT_COMMON"))
  (shadow 'object_info (find-package "NUBOT_COMMON")))
(unless (find-package "NUBOT_COMMON::OBJECT_INFO")
  (make-package "NUBOT_COMMON::OBJECT_INFO"))

(in-package "ROS")
;;//! \htmlinclude object_info.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass nubot_common::object_info
  :super ros::object
  :slots (_header _ball_pos _obs_pos _ball_know _obs_know ))

(defmethod nubot_common::object_info
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:ball_pos __ball_pos) (let (r) (dotimes (i 0) (push (instance geometry_msgs::Point :init) r)) r))
    ((:obs_pos __obs_pos) (let (r) (dotimes (i 0) (push (instance geometry_msgs::Point :init) r)) r))
    ((:ball_know __ball_know) nil)
    ((:obs_know __obs_know) nil)
    )
   (send-super :init)
   (setq _header __header)
   (setq _ball_pos __ball_pos)
   (setq _obs_pos __obs_pos)
   (setq _ball_know __ball_know)
   (setq _obs_know __obs_know)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:ball_pos
   (&rest __ball_pos)
   (if (keywordp (car __ball_pos))
       (send* _ball_pos __ball_pos)
     (progn
       (if __ball_pos (setq _ball_pos (car __ball_pos)))
       _ball_pos)))
  (:obs_pos
   (&rest __obs_pos)
   (if (keywordp (car __obs_pos))
       (send* _obs_pos __obs_pos)
     (progn
       (if __obs_pos (setq _obs_pos (car __obs_pos)))
       _obs_pos)))
  (:ball_know
   (&optional __ball_know)
   (if __ball_know (setq _ball_know __ball_know)) _ball_know)
  (:obs_know
   (&optional __obs_know)
   (if __obs_know (setq _obs_know __obs_know)) _obs_know)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; geometry_msgs/Point[] _ball_pos
    (apply #'+ (send-all _ball_pos :serialization-length)) 4
    ;; geometry_msgs/Point[] _obs_pos
    (apply #'+ (send-all _obs_pos :serialization-length)) 4
    ;; bool _ball_know
    1
    ;; bool _obs_know
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; geometry_msgs/Point[] _ball_pos
     (write-long (length _ball_pos) s)
     (dolist (elem _ball_pos)
       (send elem :serialize s)
       )
     ;; geometry_msgs/Point[] _obs_pos
     (write-long (length _obs_pos) s)
     (dolist (elem _obs_pos)
       (send elem :serialize s)
       )
     ;; bool _ball_know
       (if _ball_know (write-byte -1 s) (write-byte 0 s))
     ;; bool _obs_know
       (if _obs_know (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; geometry_msgs/Point[] _ball_pos
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _ball_pos (let (r) (dotimes (i n) (push (instance geometry_msgs::Point :init) r)) r))
     (dolist (elem- _ball_pos)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; geometry_msgs/Point[] _obs_pos
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _obs_pos (let (r) (dotimes (i n) (push (instance geometry_msgs::Point :init) r)) r))
     (dolist (elem- _obs_pos)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; bool _ball_know
     (setq _ball_know (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _obs_know
     (setq _obs_know (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get nubot_common::object_info :md5sum-) "799347d7afb7b966d6574b6b5aa1c5a4")
(setf (get nubot_common::object_info :datatype-) "nubot_common/object_info")
(setf (get nubot_common::object_info :definition-)
      "Header header
geometry_msgs/Point[] ball_pos
geometry_msgs/Point[] obs_pos
bool ball_know
bool obs_know

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

")



(provide :nubot_common/object_info "799347d7afb7b966d6574b6b5aa1c5a4")



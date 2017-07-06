;; Auto-generated. Do not edit!


(when (boundp 'nubot_common::ObstaclesInfo3d)
  (if (not (find-package "NUBOT_COMMON"))
    (make-package "NUBOT_COMMON"))
  (shadow 'ObstaclesInfo3d (find-package "NUBOT_COMMON")))
(unless (find-package "NUBOT_COMMON::OBSTACLESINFO3D")
  (make-package "NUBOT_COMMON::OBSTACLESINFO3D"))

(in-package "ROS")
;;//! \htmlinclude ObstaclesInfo3d.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass nubot_common::ObstaclesInfo3d
  :super ros::object
  :slots (_header _pos _pos_min _pos_known_3d _pos_known_2d ))

(defmethod nubot_common::ObstaclesInfo3d
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:pos __pos) (let (r) (dotimes (i 0) (push (instance nubot_common::Point3d :init) r)) r))
    ((:pos_min __pos_min) (instance nubot_common::Point3d :init))
    ((:pos_known_3d __pos_known_3d) nil)
    ((:pos_known_2d __pos_known_2d) nil)
    )
   (send-super :init)
   (setq _header __header)
   (setq _pos __pos)
   (setq _pos_min __pos_min)
   (setq _pos_known_3d __pos_known_3d)
   (setq _pos_known_2d __pos_known_2d)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:pos
   (&rest __pos)
   (if (keywordp (car __pos))
       (send* _pos __pos)
     (progn
       (if __pos (setq _pos (car __pos)))
       _pos)))
  (:pos_min
   (&rest __pos_min)
   (if (keywordp (car __pos_min))
       (send* _pos_min __pos_min)
     (progn
       (if __pos_min (setq _pos_min (car __pos_min)))
       _pos_min)))
  (:pos_known_3d
   (&optional __pos_known_3d)
   (if __pos_known_3d (setq _pos_known_3d __pos_known_3d)) _pos_known_3d)
  (:pos_known_2d
   (&optional __pos_known_2d)
   (if __pos_known_2d (setq _pos_known_2d __pos_known_2d)) _pos_known_2d)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; nubot_common/Point3d[] _pos
    (apply #'+ (send-all _pos :serialization-length)) 4
    ;; nubot_common/Point3d _pos_min
    (send _pos_min :serialization-length)
    ;; bool _pos_known_3d
    1
    ;; bool _pos_known_2d
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; nubot_common/Point3d[] _pos
     (write-long (length _pos) s)
     (dolist (elem _pos)
       (send elem :serialize s)
       )
     ;; nubot_common/Point3d _pos_min
       (send _pos_min :serialize s)
     ;; bool _pos_known_3d
       (if _pos_known_3d (write-byte -1 s) (write-byte 0 s))
     ;; bool _pos_known_2d
       (if _pos_known_2d (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; nubot_common/Point3d[] _pos
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _pos (let (r) (dotimes (i n) (push (instance nubot_common::Point3d :init) r)) r))
     (dolist (elem- _pos)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; nubot_common/Point3d _pos_min
     (send _pos_min :deserialize buf ptr-) (incf ptr- (send _pos_min :serialization-length))
   ;; bool _pos_known_3d
     (setq _pos_known_3d (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _pos_known_2d
     (setq _pos_known_2d (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get nubot_common::ObstaclesInfo3d :md5sum-) "60241ee2c1942a7ff4999dfb0ac383c8")
(setf (get nubot_common::ObstaclesInfo3d :datatype-) "nubot_common/ObstaclesInfo3d")
(setf (get nubot_common::ObstaclesInfo3d :definition-)
      "Header header
Point3d[]   pos
Point3d     pos_min

bool      pos_known_3d
bool      pos_known_2d


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
MSG: nubot_common/Point3d
float32 x
float32 y
float32 z

")



(provide :nubot_common/ObstaclesInfo3d "60241ee2c1942a7ff4999dfb0ac383c8")



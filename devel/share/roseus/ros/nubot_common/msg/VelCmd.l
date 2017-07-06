;; Auto-generated. Do not edit!


(when (boundp 'nubot_common::VelCmd)
  (if (not (find-package "NUBOT_COMMON"))
    (make-package "NUBOT_COMMON"))
  (shadow 'VelCmd (find-package "NUBOT_COMMON")))
(unless (find-package "NUBOT_COMMON::VELCMD")
  (make-package "NUBOT_COMMON::VELCMD"))

(in-package "ROS")
;;//! \htmlinclude VelCmd.msg.html


(defclass nubot_common::VelCmd
  :super ros::object
  :slots (_target _target_ori _target_vel _maxvel _maxw _robot_pos _robot_ori _move_action _rotate_acton ))

(defmethod nubot_common::VelCmd
  (:init
   (&key
    ((:target __target) (instance nubot_common::Point2d :init))
    ((:target_ori __target_ori) 0.0)
    ((:target_vel __target_vel) (instance nubot_common::Point2d :init))
    ((:maxvel __maxvel) 0.0)
    ((:maxw __maxw) 0.0)
    ((:robot_pos __robot_pos) (instance nubot_common::Point2d :init))
    ((:robot_ori __robot_ori) 0.0)
    ((:move_action __move_action) 0)
    ((:rotate_acton __rotate_acton) 0)
    )
   (send-super :init)
   (setq _target __target)
   (setq _target_ori (float __target_ori))
   (setq _target_vel __target_vel)
   (setq _maxvel (float __maxvel))
   (setq _maxw (float __maxw))
   (setq _robot_pos __robot_pos)
   (setq _robot_ori (float __robot_ori))
   (setq _move_action (round __move_action))
   (setq _rotate_acton (round __rotate_acton))
   self)
  (:target
   (&rest __target)
   (if (keywordp (car __target))
       (send* _target __target)
     (progn
       (if __target (setq _target (car __target)))
       _target)))
  (:target_ori
   (&optional __target_ori)
   (if __target_ori (setq _target_ori __target_ori)) _target_ori)
  (:target_vel
   (&rest __target_vel)
   (if (keywordp (car __target_vel))
       (send* _target_vel __target_vel)
     (progn
       (if __target_vel (setq _target_vel (car __target_vel)))
       _target_vel)))
  (:maxvel
   (&optional __maxvel)
   (if __maxvel (setq _maxvel __maxvel)) _maxvel)
  (:maxw
   (&optional __maxw)
   (if __maxw (setq _maxw __maxw)) _maxw)
  (:robot_pos
   (&rest __robot_pos)
   (if (keywordp (car __robot_pos))
       (send* _robot_pos __robot_pos)
     (progn
       (if __robot_pos (setq _robot_pos (car __robot_pos)))
       _robot_pos)))
  (:robot_ori
   (&optional __robot_ori)
   (if __robot_ori (setq _robot_ori __robot_ori)) _robot_ori)
  (:move_action
   (&optional __move_action)
   (if __move_action (setq _move_action __move_action)) _move_action)
  (:rotate_acton
   (&optional __rotate_acton)
   (if __rotate_acton (setq _rotate_acton __rotate_acton)) _rotate_acton)
  (:serialization-length
   ()
   (+
    ;; nubot_common/Point2d _target
    (send _target :serialization-length)
    ;; float32 _target_ori
    4
    ;; nubot_common/Point2d _target_vel
    (send _target_vel :serialization-length)
    ;; float32 _maxvel
    4
    ;; float32 _maxw
    4
    ;; nubot_common/Point2d _robot_pos
    (send _robot_pos :serialization-length)
    ;; float32 _robot_ori
    4
    ;; char _move_action
    1
    ;; char _rotate_acton
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; nubot_common/Point2d _target
       (send _target :serialize s)
     ;; float32 _target_ori
       (sys::poke _target_ori (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; nubot_common/Point2d _target_vel
       (send _target_vel :serialize s)
     ;; float32 _maxvel
       (sys::poke _maxvel (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _maxw
       (sys::poke _maxw (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; nubot_common/Point2d _robot_pos
       (send _robot_pos :serialize s)
     ;; float32 _robot_ori
       (sys::poke _robot_ori (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; char _move_action
       (write-byte _move_action s)
     ;; char _rotate_acton
       (write-byte _rotate_acton s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; nubot_common/Point2d _target
     (send _target :deserialize buf ptr-) (incf ptr- (send _target :serialization-length))
   ;; float32 _target_ori
     (setq _target_ori (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; nubot_common/Point2d _target_vel
     (send _target_vel :deserialize buf ptr-) (incf ptr- (send _target_vel :serialization-length))
   ;; float32 _maxvel
     (setq _maxvel (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _maxw
     (setq _maxw (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; nubot_common/Point2d _robot_pos
     (send _robot_pos :deserialize buf ptr-) (incf ptr- (send _robot_pos :serialization-length))
   ;; float32 _robot_ori
     (setq _robot_ori (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; char _move_action
     (setq _move_action (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; char _rotate_acton
     (setq _rotate_acton (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get nubot_common::VelCmd :md5sum-) "24f8f6a4f2243802ffe8d3de8b91cf6b")
(setf (get nubot_common::VelCmd :datatype-) "nubot_common/VelCmd")
(setf (get nubot_common::VelCmd :definition-)
      "Point2d target
float32 target_ori
Point2d target_vel
float32 maxvel
float32 maxw
Point2d robot_pos
float32   robot_ori
char    move_action
char    rotate_acton

================================================================================
MSG: nubot_common/Point2d
float32 x
float32 y

")



(provide :nubot_common/VelCmd "24f8f6a4f2243802ffe8d3de8b91cf6b")



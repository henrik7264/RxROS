;; Auto-generated. Do not edit!


(when (boundp 'brickpi3_ros::Range)
  (if (not (find-package "BRICKPI3_ROS"))
    (make-package "BRICKPI3_ROS"))
  (shadow 'Range (find-package "BRICKPI3_ROS")))
(unless (find-package "BRICKPI3_ROS::RANGE")
  (make-package "BRICKPI3_ROS::RANGE"))

(in-package "ROS")
;;//! \htmlinclude Range.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass brickpi3_ros::Range
  :super ros::object
  :slots (_header _range _range_min _range_max _spread_angle ))

(defmethod brickpi3_ros::Range
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:range __range) 0.0)
    ((:range_min __range_min) 0.0)
    ((:range_max __range_max) 0.0)
    ((:spread_angle __spread_angle) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _range (float __range))
   (setq _range_min (float __range_min))
   (setq _range_max (float __range_max))
   (setq _spread_angle (float __spread_angle))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:range
   (&optional __range)
   (if __range (setq _range __range)) _range)
  (:range_min
   (&optional __range_min)
   (if __range_min (setq _range_min __range_min)) _range_min)
  (:range_max
   (&optional __range_max)
   (if __range_max (setq _range_max __range_max)) _range_max)
  (:spread_angle
   (&optional __spread_angle)
   (if __spread_angle (setq _spread_angle __spread_angle)) _spread_angle)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float64 _range
    8
    ;; float64 _range_min
    8
    ;; float64 _range_max
    8
    ;; float64 _spread_angle
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float64 _range
       (sys::poke _range (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _range_min
       (sys::poke _range_min (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _range_max
       (sys::poke _range_max (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _spread_angle
       (sys::poke _spread_angle (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float64 _range
     (setq _range (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _range_min
     (setq _range_min (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _range_max
     (setq _range_max (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _spread_angle
     (setq _spread_angle (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get brickpi3_ros::Range :md5sum-) "37a3243712d0ae8a8c4863ac843a1c75")
(setf (get brickpi3_ros::Range :datatype-) "brickpi3_ros/Range")
(setf (get brickpi3_ros::Range :definition-)
      "Header header
float64 range
float64 range_min
float64 range_max
float64 spread_angle

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

")



(provide :brickpi3_ros/Range "37a3243712d0ae8a8c4863ac843a1c75")



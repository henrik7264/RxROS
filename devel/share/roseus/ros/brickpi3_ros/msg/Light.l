;; Auto-generated. Do not edit!


(when (boundp 'brickpi3_ros::Light)
  (if (not (find-package "BRICKPI3_ROS"))
    (make-package "BRICKPI3_ROS"))
  (shadow 'Light (find-package "BRICKPI3_ROS")))
(unless (find-package "BRICKPI3_ROS::LIGHT")
  (make-package "BRICKPI3_ROS::LIGHT"))

(in-package "ROS")
;;//! \htmlinclude Light.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass brickpi3_ros::Light
  :super ros::object
  :slots (_header _intensity ))

(defmethod brickpi3_ros::Light
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:intensity __intensity) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _intensity (float __intensity))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:intensity
   (&optional __intensity)
   (if __intensity (setq _intensity __intensity)) _intensity)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float64 _intensity
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float64 _intensity
       (sys::poke _intensity (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float64 _intensity
     (setq _intensity (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get brickpi3_ros::Light :md5sum-) "ff9119148a4a3f3a31d0a976a2a2cd04")
(setf (get brickpi3_ros::Light :datatype-) "brickpi3_ros/Light")
(setf (get brickpi3_ros::Light :definition-)
      "Header header
float64 intensity

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



(provide :brickpi3_ros/Light "ff9119148a4a3f3a31d0a976a2a2cd04")



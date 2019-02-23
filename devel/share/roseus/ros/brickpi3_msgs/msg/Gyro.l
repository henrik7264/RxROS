;; Auto-generated. Do not edit!


(when (boundp 'brickpi3_msgs::Gyro)
  (if (not (find-package "BRICKPI3_MSGS"))
    (make-package "BRICKPI3_MSGS"))
  (shadow 'Gyro (find-package "BRICKPI3_MSGS")))
(unless (find-package "BRICKPI3_MSGS::GYRO")
  (make-package "BRICKPI3_MSGS::GYRO"))

(in-package "ROS")
;;//! \htmlinclude Gyro.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass brickpi3_msgs::Gyro
  :super ros::object
  :slots (_header _calibration_offset _angular_velocity _angular_velocity_covariance ))

(defmethod brickpi3_msgs::Gyro
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:calibration_offset __calibration_offset) (instance geometry_msgs::Vector3 :init))
    ((:angular_velocity __angular_velocity) (instance geometry_msgs::Vector3 :init))
    ((:angular_velocity_covariance __angular_velocity_covariance) (make-array 9 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _header __header)
   (setq _calibration_offset __calibration_offset)
   (setq _angular_velocity __angular_velocity)
   (setq _angular_velocity_covariance __angular_velocity_covariance)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:calibration_offset
   (&rest __calibration_offset)
   (if (keywordp (car __calibration_offset))
       (send* _calibration_offset __calibration_offset)
     (progn
       (if __calibration_offset (setq _calibration_offset (car __calibration_offset)))
       _calibration_offset)))
  (:angular_velocity
   (&rest __angular_velocity)
   (if (keywordp (car __angular_velocity))
       (send* _angular_velocity __angular_velocity)
     (progn
       (if __angular_velocity (setq _angular_velocity (car __angular_velocity)))
       _angular_velocity)))
  (:angular_velocity_covariance
   (&optional __angular_velocity_covariance)
   (if __angular_velocity_covariance (setq _angular_velocity_covariance __angular_velocity_covariance)) _angular_velocity_covariance)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; geometry_msgs/Vector3 _calibration_offset
    (send _calibration_offset :serialization-length)
    ;; geometry_msgs/Vector3 _angular_velocity
    (send _angular_velocity :serialization-length)
    ;; float64[9] _angular_velocity_covariance
    (* 8    9)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; geometry_msgs/Vector3 _calibration_offset
       (send _calibration_offset :serialize s)
     ;; geometry_msgs/Vector3 _angular_velocity
       (send _angular_velocity :serialize s)
     ;; float64[9] _angular_velocity_covariance
     (dotimes (i 9)
       (sys::poke (elt _angular_velocity_covariance i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; geometry_msgs/Vector3 _calibration_offset
     (send _calibration_offset :deserialize buf ptr-) (incf ptr- (send _calibration_offset :serialization-length))
   ;; geometry_msgs/Vector3 _angular_velocity
     (send _angular_velocity :deserialize buf ptr-) (incf ptr- (send _angular_velocity :serialization-length))
   ;; float64[9] _angular_velocity_covariance
   (dotimes (i (length _angular_velocity_covariance))
     (setf (elt _angular_velocity_covariance i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;;
   self)
  )

(setf (get brickpi3_msgs::Gyro :md5sum-) "e8c3b3c062e5c0998e1d03e28394cf83")
(setf (get brickpi3_msgs::Gyro :datatype-) "brickpi3_msgs/Gyro")
(setf (get brickpi3_msgs::Gyro :definition-)
      "Header header
geometry_msgs/Vector3 calibration_offset
geometry_msgs/Vector3 angular_velocity
float64[9] angular_velocity_covariance
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
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
")



(provide :brickpi3_msgs/Gyro "e8c3b3c062e5c0998e1d03e28394cf83")



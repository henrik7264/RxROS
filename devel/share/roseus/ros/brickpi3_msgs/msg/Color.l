;; Auto-generated. Do not edit!


(when (boundp 'brickpi3_msgs::Color)
  (if (not (find-package "BRICKPI3_MSGS"))
    (make-package "BRICKPI3_MSGS"))
  (shadow 'Color (find-package "BRICKPI3_MSGS")))
(unless (find-package "BRICKPI3_MSGS::COLOR")
  (make-package "BRICKPI3_MSGS::COLOR"))

(in-package "ROS")
;;//! \htmlinclude Color.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass brickpi3_msgs::Color
  :super ros::object
  :slots (_header _intensity _r _g _b ))

(defmethod brickpi3_msgs::Color
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:intensity __intensity) 0.0)
    ((:r __r) 0.0)
    ((:g __g) 0.0)
    ((:b __b) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _intensity (float __intensity))
   (setq _r (float __r))
   (setq _g (float __g))
   (setq _b (float __b))
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
  (:r
   (&optional __r)
   (if __r (setq _r __r)) _r)
  (:g
   (&optional __g)
   (if __g (setq _g __g)) _g)
  (:b
   (&optional __b)
   (if __b (setq _b __b)) _b)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float64 _intensity
    8
    ;; float64 _r
    8
    ;; float64 _g
    8
    ;; float64 _b
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
     ;; float64 _r
       (sys::poke _r (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _g
       (sys::poke _g (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _b
       (sys::poke _b (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float64 _intensity
     (setq _intensity (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _r
     (setq _r (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _g
     (setq _g (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _b
     (setq _b (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get brickpi3_msgs::Color :md5sum-) "a62a6387c4711f26d641ffa6e12a86e8")
(setf (get brickpi3_msgs::Color :datatype-) "brickpi3_msgs/Color")
(setf (get brickpi3_msgs::Color :definition-)
      "Header header
float64 intensity
float64 r
float64 g
float64 b
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
string frame_id

")



(provide :brickpi3_msgs/Color "a62a6387c4711f26d641ffa6e12a86e8")



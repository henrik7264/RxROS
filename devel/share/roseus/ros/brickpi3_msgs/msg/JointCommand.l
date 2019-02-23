;; Auto-generated. Do not edit!


(when (boundp 'brickpi3_msgs::JointCommand)
  (if (not (find-package "BRICKPI3_MSGS"))
    (make-package "BRICKPI3_MSGS"))
  (shadow 'JointCommand (find-package "BRICKPI3_MSGS")))
(unless (find-package "BRICKPI3_MSGS::JOINTCOMMAND")
  (make-package "BRICKPI3_MSGS::JOINTCOMMAND"))

(in-package "ROS")
;;//! \htmlinclude JointCommand.msg.html


(defclass brickpi3_msgs::JointCommand
  :super ros::object
  :slots (_name _effort ))

(defmethod brickpi3_msgs::JointCommand
  (:init
   (&key
    ((:name __name) "")
    ((:effort __effort) 0.0)
    )
   (send-super :init)
   (setq _name (string __name))
   (setq _effort (float __effort))
   self)
  (:name
   (&optional __name)
   (if __name (setq _name __name)) _name)
  (:effort
   (&optional __effort)
   (if __effort (setq _effort __effort)) _effort)
  (:serialization-length
   ()
   (+
    ;; string _name
    4 (length _name)
    ;; float64 _effort
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _name
       (write-long (length _name) s) (princ _name s)
     ;; float64 _effort
       (sys::poke _effort (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float64 _effort
     (setq _effort (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get brickpi3_msgs::JointCommand :md5sum-) "9de38d5a93df00cd9489becfca3e874a")
(setf (get brickpi3_msgs::JointCommand :datatype-) "brickpi3_msgs/JointCommand")
(setf (get brickpi3_msgs::JointCommand :definition-)
      "string name
float64 effort

")



(provide :brickpi3_msgs/JointCommand "9de38d5a93df00cd9489becfca3e874a")



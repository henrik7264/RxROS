; Auto-generated. Do not edit!


(cl:in-package brickpi3_ros-msg)


;//! \htmlinclude JointCommand.msg.html

(cl:defclass <JointCommand> (roslisp-msg-protocol:ros-message)
  ((name
    :reader name
    :initarg :name
    :type cl:string
    :initform "")
   (effort
    :reader effort
    :initarg :effort
    :type cl:float
    :initform 0.0))
)

(cl:defclass JointCommand (<JointCommand>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <JointCommand>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'JointCommand)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name brickpi3_ros-msg:<JointCommand> is deprecated: use brickpi3_ros-msg:JointCommand instead.")))

(cl:ensure-generic-function 'name-val :lambda-list '(m))
(cl:defmethod name-val ((m <JointCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader brickpi3_ros-msg:name-val is deprecated.  Use brickpi3_ros-msg:name instead.")
  (name m))

(cl:ensure-generic-function 'effort-val :lambda-list '(m))
(cl:defmethod effort-val ((m <JointCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader brickpi3_ros-msg:effort-val is deprecated.  Use brickpi3_ros-msg:effort instead.")
  (effort m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <JointCommand>) ostream)
  "Serializes a message object of type '<JointCommand>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'name))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'effort))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <JointCommand>) istream)
  "Deserializes a message object of type '<JointCommand>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'effort) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<JointCommand>)))
  "Returns string type for a message object of type '<JointCommand>"
  "brickpi3_ros/JointCommand")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'JointCommand)))
  "Returns string type for a message object of type 'JointCommand"
  "brickpi3_ros/JointCommand")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<JointCommand>)))
  "Returns md5sum for a message object of type '<JointCommand>"
  "9de38d5a93df00cd9489becfca3e874a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'JointCommand)))
  "Returns md5sum for a message object of type 'JointCommand"
  "9de38d5a93df00cd9489becfca3e874a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<JointCommand>)))
  "Returns full string definition for message of type '<JointCommand>"
  (cl:format cl:nil "string name~%float64 effort~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'JointCommand)))
  "Returns full string definition for message of type 'JointCommand"
  (cl:format cl:nil "string name~%float64 effort~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <JointCommand>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'name))
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <JointCommand>))
  "Converts a ROS message object to a list"
  (cl:list 'JointCommand
    (cl:cons ':name (name msg))
    (cl:cons ':effort (effort msg))
))

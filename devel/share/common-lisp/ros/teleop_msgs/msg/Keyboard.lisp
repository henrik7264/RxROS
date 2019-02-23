; Auto-generated. Do not edit!


(cl:in-package teleop_msgs-msg)


;//! \htmlinclude Keyboard.msg.html

(cl:defclass <Keyboard> (roslisp-msg-protocol:ros-message)
  ((time
    :reader time
    :initarg :time
    :type cl:real
    :initform 0)
   (event
    :reader event
    :initarg :event
    :type cl:fixnum
    :initform 0))
)

(cl:defclass Keyboard (<Keyboard>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Keyboard>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Keyboard)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name teleop_msgs-msg:<Keyboard> is deprecated: use teleop_msgs-msg:Keyboard instead.")))

(cl:ensure-generic-function 'time-val :lambda-list '(m))
(cl:defmethod time-val ((m <Keyboard>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader teleop_msgs-msg:time-val is deprecated.  Use teleop_msgs-msg:time instead.")
  (time m))

(cl:ensure-generic-function 'event-val :lambda-list '(m))
(cl:defmethod event-val ((m <Keyboard>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader teleop_msgs-msg:event-val is deprecated.  Use teleop_msgs-msg:event instead.")
  (event m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Keyboard>) ostream)
  "Serializes a message object of type '<Keyboard>"
  (cl:let ((__sec (cl:floor (cl:slot-value msg 'time)))
        (__nsec (cl:round (cl:* 1e9 (cl:- (cl:slot-value msg 'time) (cl:floor (cl:slot-value msg 'time)))))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 0) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __nsec) ostream))
  (cl:let* ((signed (cl:slot-value msg 'event)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Keyboard>) istream)
  "Deserializes a message object of type '<Keyboard>"
    (cl:let ((__sec 0) (__nsec 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 0) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __nsec) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'time) (cl:+ (cl:coerce __sec 'cl:double-float) (cl:/ __nsec 1e9))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'event) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Keyboard>)))
  "Returns string type for a message object of type '<Keyboard>"
  "teleop_msgs/Keyboard")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Keyboard)))
  "Returns string type for a message object of type 'Keyboard"
  "teleop_msgs/Keyboard")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Keyboard>)))
  "Returns md5sum for a message object of type '<Keyboard>"
  "e5fbe2cc6f38678d510728a2dcb2ff75")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Keyboard)))
  "Returns md5sum for a message object of type 'Keyboard"
  "e5fbe2cc6f38678d510728a2dcb2ff75")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Keyboard>)))
  "Returns full string definition for message of type '<Keyboard>"
  (cl:format cl:nil "time time~%int8 event~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Keyboard)))
  "Returns full string definition for message of type 'Keyboard"
  (cl:format cl:nil "time time~%int8 event~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Keyboard>))
  (cl:+ 0
     8
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Keyboard>))
  "Converts a ROS message object to a list"
  (cl:list 'Keyboard
    (cl:cons ':time (time msg))
    (cl:cons ':event (event msg))
))

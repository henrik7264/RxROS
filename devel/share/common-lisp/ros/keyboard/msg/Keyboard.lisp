; Auto-generated. Do not edit!


(cl:in-package keyboard-msg)


;//! \htmlinclude Keyboard.msg.html

(cl:defclass <Keyboard> (roslisp-msg-protocol:ros-message)
  ((time
    :reader time
    :initarg :time
    :type cl:real
    :initform 0)
   (key
    :reader key
    :initarg :key
    :type cl:fixnum
    :initform 0))
)

(cl:defclass Keyboard (<Keyboard>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Keyboard>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Keyboard)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name keyboard-msg:<Keyboard> is deprecated: use keyboard-msg:Keyboard instead.")))

(cl:ensure-generic-function 'time-val :lambda-list '(m))
(cl:defmethod time-val ((m <Keyboard>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader keyboard-msg:time-val is deprecated.  Use keyboard-msg:time instead.")
  (time m))

(cl:ensure-generic-function 'key-val :lambda-list '(m))
(cl:defmethod key-val ((m <Keyboard>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader keyboard-msg:key-val is deprecated.  Use keyboard-msg:key instead.")
  (key m))
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
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'key)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'key)) ostream)
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
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'key)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'key)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Keyboard>)))
  "Returns string type for a message object of type '<Keyboard>"
  "keyboard/Keyboard")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Keyboard)))
  "Returns string type for a message object of type 'Keyboard"
  "keyboard/Keyboard")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Keyboard>)))
  "Returns md5sum for a message object of type '<Keyboard>"
  "aa33d4f1ca40c17b9cc7a9d013da833e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Keyboard)))
  "Returns md5sum for a message object of type 'Keyboard"
  "aa33d4f1ca40c17b9cc7a9d013da833e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Keyboard>)))
  "Returns full string definition for message of type '<Keyboard>"
  (cl:format cl:nil "time time~%uint16 key~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Keyboard)))
  "Returns full string definition for message of type 'Keyboard"
  (cl:format cl:nil "time time~%uint16 key~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Keyboard>))
  (cl:+ 0
     8
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Keyboard>))
  "Converts a ROS message object to a list"
  (cl:list 'Keyboard
    (cl:cons ':time (time msg))
    (cl:cons ':key (key msg))
))

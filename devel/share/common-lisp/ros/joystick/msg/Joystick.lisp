; Auto-generated. Do not edit!


(cl:in-package joystick-msg)


;//! \htmlinclude Joystick.msg.html

(cl:defclass <Joystick> (roslisp-msg-protocol:ros-message)
  ((time
    :reader time
    :initarg :time
    :type cl:real
    :initform 0)
   (value
    :reader value
    :initarg :value
    :type cl:fixnum
    :initform 0)
   (type
    :reader type
    :initarg :type
    :type cl:fixnum
    :initform 0)
   (number
    :reader number
    :initarg :number
    :type cl:fixnum
    :initform 0))
)

(cl:defclass Joystick (<Joystick>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Joystick>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Joystick)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name joystick-msg:<Joystick> is deprecated: use joystick-msg:Joystick instead.")))

(cl:ensure-generic-function 'time-val :lambda-list '(m))
(cl:defmethod time-val ((m <Joystick>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader joystick-msg:time-val is deprecated.  Use joystick-msg:time instead.")
  (time m))

(cl:ensure-generic-function 'value-val :lambda-list '(m))
(cl:defmethod value-val ((m <Joystick>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader joystick-msg:value-val is deprecated.  Use joystick-msg:value instead.")
  (value m))

(cl:ensure-generic-function 'type-val :lambda-list '(m))
(cl:defmethod type-val ((m <Joystick>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader joystick-msg:type-val is deprecated.  Use joystick-msg:type instead.")
  (type m))

(cl:ensure-generic-function 'number-val :lambda-list '(m))
(cl:defmethod number-val ((m <Joystick>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader joystick-msg:number-val is deprecated.  Use joystick-msg:number instead.")
  (number m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Joystick>) ostream)
  "Serializes a message object of type '<Joystick>"
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
  (cl:let* ((signed (cl:slot-value msg 'value)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'type)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'number)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Joystick>) istream)
  "Deserializes a message object of type '<Joystick>"
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
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'value) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'type) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'number) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Joystick>)))
  "Returns string type for a message object of type '<Joystick>"
  "joystick/Joystick")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Joystick)))
  "Returns string type for a message object of type 'Joystick"
  "joystick/Joystick")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Joystick>)))
  "Returns md5sum for a message object of type '<Joystick>"
  "053a7b1f7f659589125a990814760aa6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Joystick)))
  "Returns md5sum for a message object of type 'Joystick"
  "053a7b1f7f659589125a990814760aa6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Joystick>)))
  "Returns full string definition for message of type '<Joystick>"
  (cl:format cl:nil "time time~%int16 value~%int8 type~%int8 number~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Joystick)))
  "Returns full string definition for message of type 'Joystick"
  (cl:format cl:nil "time time~%int16 value~%int8 type~%int8 number~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Joystick>))
  (cl:+ 0
     8
     2
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Joystick>))
  "Converts a ROS message object to a list"
  (cl:list 'Joystick
    (cl:cons ':time (time msg))
    (cl:cons ':value (value msg))
    (cl:cons ':type (type msg))
    (cl:cons ':number (number msg))
))

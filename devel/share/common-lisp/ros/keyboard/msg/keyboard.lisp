; Auto-generated. Do not edit!


(cl:in-package keyboard-msg)


;//! \htmlinclude keyboard.msg.html

(cl:defclass <keyboard> (roslisp-msg-protocol:ros-message)
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

(cl:defclass keyboard (<keyboard>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <keyboard>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'keyboard)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name keyboard-msg:<keyboard> is deprecated: use keyboard-msg:keyboard instead.")))

(cl:ensure-generic-function 'time-val :lambda-list '(m))
(cl:defmethod time-val ((m <keyboard>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader keyboard-msg:time-val is deprecated.  Use keyboard-msg:time instead.")
  (time m))

(cl:ensure-generic-function 'event-val :lambda-list '(m))
(cl:defmethod event-val ((m <keyboard>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader keyboard-msg:event-val is deprecated.  Use keyboard-msg:event instead.")
  (event m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <keyboard>) ostream)
  "Serializes a message object of type '<keyboard>"
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <keyboard>) istream)
  "Deserializes a message object of type '<keyboard>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<keyboard>)))
  "Returns string type for a message object of type '<keyboard>"
  "keyboard/keyboard")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'keyboard)))
  "Returns string type for a message object of type 'keyboard"
  "keyboard/keyboard")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<keyboard>)))
  "Returns md5sum for a message object of type '<keyboard>"
  "e5fbe2cc6f38678d510728a2dcb2ff75")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'keyboard)))
  "Returns md5sum for a message object of type 'keyboard"
  "e5fbe2cc6f38678d510728a2dcb2ff75")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<keyboard>)))
  "Returns full string definition for message of type '<keyboard>"
  (cl:format cl:nil "time time~%int8 event~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'keyboard)))
  "Returns full string definition for message of type 'keyboard"
  (cl:format cl:nil "time time~%int8 event~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <keyboard>))
  (cl:+ 0
     8
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <keyboard>))
  "Converts a ROS message object to a list"
  (cl:list 'keyboard
    (cl:cons ':time (time msg))
    (cl:cons ':event (event msg))
))

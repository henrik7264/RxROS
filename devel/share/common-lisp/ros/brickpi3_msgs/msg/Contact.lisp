; Auto-generated. Do not edit!


(cl:in-package brickpi3_msgs-msg)


;//! \htmlinclude Contact.msg.html

(cl:defclass <Contact> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (contact
    :reader contact
    :initarg :contact
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass Contact (<Contact>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Contact>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Contact)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name brickpi3_msgs-msg:<Contact> is deprecated: use brickpi3_msgs-msg:Contact instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <Contact>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader brickpi3_msgs-msg:header-val is deprecated.  Use brickpi3_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'contact-val :lambda-list '(m))
(cl:defmethod contact-val ((m <Contact>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader brickpi3_msgs-msg:contact-val is deprecated.  Use brickpi3_msgs-msg:contact instead.")
  (contact m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Contact>) ostream)
  "Serializes a message object of type '<Contact>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'contact) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Contact>) istream)
  "Deserializes a message object of type '<Contact>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:slot-value msg 'contact) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Contact>)))
  "Returns string type for a message object of type '<Contact>"
  "brickpi3_msgs/Contact")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Contact)))
  "Returns string type for a message object of type 'Contact"
  "brickpi3_msgs/Contact")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Contact>)))
  "Returns md5sum for a message object of type '<Contact>"
  "771b237034839b48d5de401dc68aaf43")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Contact)))
  "Returns md5sum for a message object of type 'Contact"
  "771b237034839b48d5de401dc68aaf43")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Contact>)))
  "Returns full string definition for message of type '<Contact>"
  (cl:format cl:nil "Header header~%bool contact~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Contact)))
  "Returns full string definition for message of type 'Contact"
  (cl:format cl:nil "Header header~%bool contact~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Contact>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Contact>))
  "Converts a ROS message object to a list"
  (cl:list 'Contact
    (cl:cons ':header (header msg))
    (cl:cons ':contact (contact msg))
))

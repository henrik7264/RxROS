;; Auto-generated. Do not edit!


(when (boundp 'joystick::Joystick)
  (if (not (find-package "JOYSTICK"))
    (make-package "JOYSTICK"))
  (shadow 'Joystick (find-package "JOYSTICK")))
(unless (find-package "JOYSTICK::JOYSTICK")
  (make-package "JOYSTICK::JOYSTICK"))

(in-package "ROS")
;;//! \htmlinclude Joystick.msg.html


(defclass joystick::Joystick
  :super ros::object
  :slots (_time _value _type _number ))

(defmethod joystick::Joystick
  (:init
   (&key
    ((:time __time) (instance ros::time :init))
    ((:value __value) 0)
    ((:type __type) 0)
    ((:number __number) 0)
    )
   (send-super :init)
   (setq _time __time)
   (setq _value (round __value))
   (setq _type (round __type))
   (setq _number (round __number))
   self)
  (:time
   (&optional __time)
   (if __time (setq _time __time)) _time)
  (:value
   (&optional __value)
   (if __value (setq _value __value)) _value)
  (:type
   (&optional __type)
   (if __type (setq _type __type)) _type)
  (:number
   (&optional __number)
   (if __number (setq _number __number)) _number)
  (:serialization-length
   ()
   (+
    ;; time _time
    8
    ;; int16 _value
    2
    ;; int8 _type
    1
    ;; int8 _number
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; time _time
       (write-long (send _time :sec) s) (write-long (send _time :nsec) s)
     ;; int16 _value
       (write-word _value s)
     ;; int8 _type
       (write-byte _type s)
     ;; int8 _number
       (write-byte _number s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; time _time
     (send _time :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _time :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int16 _value
     (setq _value (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int8 _type
     (setq _type (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _type 127) (setq _type (- _type 256)))
   ;; int8 _number
     (setq _number (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _number 127) (setq _number (- _number 256)))
   ;;
   self)
  )

(setf (get joystick::Joystick :md5sum-) "053a7b1f7f659589125a990814760aa6")
(setf (get joystick::Joystick :datatype-) "joystick/Joystick")
(setf (get joystick::Joystick :definition-)
      "time time
int16 value
int8 type
int8 number

")



(provide :joystick/Joystick "053a7b1f7f659589125a990814760aa6")



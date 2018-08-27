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
  :slots (_time _event ))

(defmethod joystick::Joystick
  (:init
   (&key
    ((:time __time) (instance ros::time :init))
    ((:event __event) 0)
    )
   (send-super :init)
   (setq _time __time)
   (setq _event (round __event))
   self)
  (:time
   (&optional __time)
   (if __time (setq _time __time)) _time)
  (:event
   (&optional __event)
   (if __event (setq _event __event)) _event)
  (:serialization-length
   ()
   (+
    ;; time _time
    8
    ;; int8 _event
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; time _time
       (write-long (send _time :sec) s) (write-long (send _time :nsec) s)
     ;; int8 _event
       (write-byte _event s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; time _time
     (send _time :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _time :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int8 _event
     (setq _event (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _event 127) (setq _event (- _event 256)))
   ;;
   self)
  )

(setf (get joystick::Joystick :md5sum-) "e5fbe2cc6f38678d510728a2dcb2ff75")
(setf (get joystick::Joystick :datatype-) "joystick/Joystick")
(setf (get joystick::Joystick :definition-)
      "time time
int8 event

")



(provide :joystick/Joystick "e5fbe2cc6f38678d510728a2dcb2ff75")



;; Auto-generated. Do not edit!


(when (boundp 'keyboard::Keyboard)
  (if (not (find-package "KEYBOARD"))
    (make-package "KEYBOARD"))
  (shadow 'Keyboard (find-package "KEYBOARD")))
(unless (find-package "KEYBOARD::KEYBOARD")
  (make-package "KEYBOARD::KEYBOARD"))

(in-package "ROS")
;;//! \htmlinclude Keyboard.msg.html


(defclass keyboard::Keyboard
  :super ros::object
  :slots (_time _key ))

(defmethod keyboard::Keyboard
  (:init
   (&key
    ((:time __time) (instance ros::time :init))
    ((:key __key) 0)
    )
   (send-super :init)
   (setq _time __time)
   (setq _key (round __key))
   self)
  (:time
   (&optional __time)
   (if __time (setq _time __time)) _time)
  (:key
   (&optional __key)
   (if __key (setq _key __key)) _key)
  (:serialization-length
   ()
   (+
    ;; time _time
    8
    ;; uint16 _key
    2
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; time _time
       (write-long (send _time :sec) s) (write-long (send _time :nsec) s)
     ;; uint16 _key
       (write-word _key s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; time _time
     (send _time :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _time :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint16 _key
     (setq _key (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;;
   self)
  )

(setf (get keyboard::Keyboard :md5sum-) "aa33d4f1ca40c17b9cc7a9d013da833e")
(setf (get keyboard::Keyboard :datatype-) "keyboard/Keyboard")
(setf (get keyboard::Keyboard :definition-)
      "time time
uint16 key

")



(provide :keyboard/Keyboard "aa33d4f1ca40c17b9cc7a9d013da833e")



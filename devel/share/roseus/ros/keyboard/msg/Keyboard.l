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
  :slots (_time _event ))

(defmethod keyboard::Keyboard
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
    ;; uint8 _event
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; time _time
       (write-long (send _time :sec) s) (write-long (send _time :nsec) s)
     ;; uint8 _event
       (write-byte _event s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; time _time
     (send _time :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _time :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint8 _event
     (setq _event (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get keyboard::Keyboard :md5sum-) "0cc7a080d169ac52b2bb65f31811058d")
(setf (get keyboard::Keyboard :datatype-) "keyboard/Keyboard")
(setf (get keyboard::Keyboard :definition-)
      "time time
uint8 event

")



(provide :keyboard/Keyboard "0cc7a080d169ac52b2bb65f31811058d")



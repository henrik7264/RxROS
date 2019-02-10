
(cl:in-package :asdf)

(defsystem "teleop-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Joystick" :depends-on ("_package_Joystick"))
    (:file "_package_Joystick" :depends-on ("_package"))
    (:file "Keyboard" :depends-on ("_package_Keyboard"))
    (:file "_package_Keyboard" :depends-on ("_package"))
  ))
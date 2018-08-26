
(cl:in-package :asdf)

(defsystem "joystick-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Joystick" :depends-on ("_package_Joystick"))
    (:file "_package_Joystick" :depends-on ("_package"))
  ))
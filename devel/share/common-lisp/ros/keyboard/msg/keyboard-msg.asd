
(cl:in-package :asdf)

(defsystem "keyboard-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "keyboard" :depends-on ("_package_keyboard"))
    (:file "_package_keyboard" :depends-on ("_package"))
  ))
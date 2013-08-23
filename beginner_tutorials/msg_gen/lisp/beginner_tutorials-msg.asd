
(cl:in-package :asdf)

(defsystem "beginner_tutorials-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "wrenchData" :depends-on ("_package_wrenchData"))
    (:file "_package_wrenchData" :depends-on ("_package"))
  ))
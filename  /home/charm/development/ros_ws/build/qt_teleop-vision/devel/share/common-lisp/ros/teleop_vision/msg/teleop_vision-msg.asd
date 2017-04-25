
(cl:in-package :asdf)

(defsystem "teleop_vision-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "TaskState" :depends-on ("_package_TaskState"))
    (:file "_package_TaskState" :depends-on ("_package"))
  ))
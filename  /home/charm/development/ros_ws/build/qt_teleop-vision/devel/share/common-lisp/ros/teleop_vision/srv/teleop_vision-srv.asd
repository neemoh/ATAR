
(cl:in-package :asdf)

(defsystem "teleop_vision-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "CalculateStereoCamsTransfromFromTopics" :depends-on ("_package_CalculateStereoCamsTransfromFromTopics"))
    (:file "_package_CalculateStereoCamsTransfromFromTopics" :depends-on ("_package"))
  ))
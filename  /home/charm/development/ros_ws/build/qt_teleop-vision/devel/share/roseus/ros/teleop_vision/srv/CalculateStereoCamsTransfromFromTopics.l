;; Auto-generated. Do not edit!


(when (boundp 'teleop_vision::CalculateStereoCamsTransfromFromTopics)
  (if (not (find-package "TELEOP_VISION"))
    (make-package "TELEOP_VISION"))
  (shadow 'CalculateStereoCamsTransfromFromTopics (find-package "TELEOP_VISION")))
(unless (find-package "TELEOP_VISION::CALCULATESTEREOCAMSTRANSFROMFROMTOPICS")
  (make-package "TELEOP_VISION::CALCULATESTEREOCAMSTRANSFROMFROMTOPICS"))
(unless (find-package "TELEOP_VISION::CALCULATESTEREOCAMSTRANSFROMFROMTOPICSREQUEST")
  (make-package "TELEOP_VISION::CALCULATESTEREOCAMSTRANSFROMFROMTOPICSREQUEST"))
(unless (find-package "TELEOP_VISION::CALCULATESTEREOCAMSTRANSFROMFROMTOPICSRESPONSE")
  (make-package "TELEOP_VISION::CALCULATESTEREOCAMSTRANSFROMFROMTOPICSRESPONSE"))

(in-package "ROS")



(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass teleop_vision::CalculateStereoCamsTransfromFromTopicsRequest
  :super ros::object
  :slots (_cam_1_pose_topic_name _cam_2_pose_topic_name ))

(defmethod teleop_vision::CalculateStereoCamsTransfromFromTopicsRequest
  (:init
   (&key
    ((:cam_1_pose_topic_name __cam_1_pose_topic_name) "")
    ((:cam_2_pose_topic_name __cam_2_pose_topic_name) "")
    )
   (send-super :init)
   (setq _cam_1_pose_topic_name (string __cam_1_pose_topic_name))
   (setq _cam_2_pose_topic_name (string __cam_2_pose_topic_name))
   self)
  (:cam_1_pose_topic_name
   (&optional __cam_1_pose_topic_name)
   (if __cam_1_pose_topic_name (setq _cam_1_pose_topic_name __cam_1_pose_topic_name)) _cam_1_pose_topic_name)
  (:cam_2_pose_topic_name
   (&optional __cam_2_pose_topic_name)
   (if __cam_2_pose_topic_name (setq _cam_2_pose_topic_name __cam_2_pose_topic_name)) _cam_2_pose_topic_name)
  (:serialization-length
   ()
   (+
    ;; string _cam_1_pose_topic_name
    4 (length _cam_1_pose_topic_name)
    ;; string _cam_2_pose_topic_name
    4 (length _cam_2_pose_topic_name)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _cam_1_pose_topic_name
       (write-long (length _cam_1_pose_topic_name) s) (princ _cam_1_pose_topic_name s)
     ;; string _cam_2_pose_topic_name
       (write-long (length _cam_2_pose_topic_name) s) (princ _cam_2_pose_topic_name s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _cam_1_pose_topic_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _cam_1_pose_topic_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _cam_2_pose_topic_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _cam_2_pose_topic_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass teleop_vision::CalculateStereoCamsTransfromFromTopicsResponse
  :super ros::object
  :slots (_cam_1_to_cam_2_pose ))

(defmethod teleop_vision::CalculateStereoCamsTransfromFromTopicsResponse
  (:init
   (&key
    ((:cam_1_to_cam_2_pose __cam_1_to_cam_2_pose) (instance geometry_msgs::Pose :init))
    )
   (send-super :init)
   (setq _cam_1_to_cam_2_pose __cam_1_to_cam_2_pose)
   self)
  (:cam_1_to_cam_2_pose
   (&rest __cam_1_to_cam_2_pose)
   (if (keywordp (car __cam_1_to_cam_2_pose))
       (send* _cam_1_to_cam_2_pose __cam_1_to_cam_2_pose)
     (progn
       (if __cam_1_to_cam_2_pose (setq _cam_1_to_cam_2_pose (car __cam_1_to_cam_2_pose)))
       _cam_1_to_cam_2_pose)))
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Pose _cam_1_to_cam_2_pose
    (send _cam_1_to_cam_2_pose :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Pose _cam_1_to_cam_2_pose
       (send _cam_1_to_cam_2_pose :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Pose _cam_1_to_cam_2_pose
     (send _cam_1_to_cam_2_pose :deserialize buf ptr-) (incf ptr- (send _cam_1_to_cam_2_pose :serialization-length))
   ;;
   self)
  )

(defclass teleop_vision::CalculateStereoCamsTransfromFromTopics
  :super ros::object
  :slots ())

(setf (get teleop_vision::CalculateStereoCamsTransfromFromTopics :md5sum-) "28012a097e6af14c818b4f0effba0194")
(setf (get teleop_vision::CalculateStereoCamsTransfromFromTopics :datatype-) "teleop_vision/CalculateStereoCamsTransfromFromTopics")
(setf (get teleop_vision::CalculateStereoCamsTransfromFromTopics :request) teleop_vision::CalculateStereoCamsTransfromFromTopicsRequest)
(setf (get teleop_vision::CalculateStereoCamsTransfromFromTopics :response) teleop_vision::CalculateStereoCamsTransfromFromTopicsResponse)

(defmethod teleop_vision::CalculateStereoCamsTransfromFromTopicsRequest
  (:response () (instance teleop_vision::CalculateStereoCamsTransfromFromTopicsResponse :init)))

(setf (get teleop_vision::CalculateStereoCamsTransfromFromTopicsRequest :md5sum-) "28012a097e6af14c818b4f0effba0194")
(setf (get teleop_vision::CalculateStereoCamsTransfromFromTopicsRequest :datatype-) "teleop_vision/CalculateStereoCamsTransfromFromTopicsRequest")
(setf (get teleop_vision::CalculateStereoCamsTransfromFromTopicsRequest :definition-)
      "string cam_1_pose_topic_name
string cam_2_pose_topic_name
---
geometry_msgs/Pose cam_1_to_cam_2_pose

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
")

(setf (get teleop_vision::CalculateStereoCamsTransfromFromTopicsResponse :md5sum-) "28012a097e6af14c818b4f0effba0194")
(setf (get teleop_vision::CalculateStereoCamsTransfromFromTopicsResponse :datatype-) "teleop_vision/CalculateStereoCamsTransfromFromTopicsResponse")
(setf (get teleop_vision::CalculateStereoCamsTransfromFromTopicsResponse :definition-)
      "string cam_1_pose_topic_name
string cam_2_pose_topic_name
---
geometry_msgs/Pose cam_1_to_cam_2_pose

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
")



(provide :teleop_vision/CalculateStereoCamsTransfromFromTopics "28012a097e6af14c818b4f0effba0194")



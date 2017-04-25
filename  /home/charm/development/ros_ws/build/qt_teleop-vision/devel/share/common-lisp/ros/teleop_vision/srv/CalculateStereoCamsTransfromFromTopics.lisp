; Auto-generated. Do not edit!


(cl:in-package teleop_vision-srv)


;//! \htmlinclude CalculateStereoCamsTransfromFromTopics-request.msg.html

(cl:defclass <CalculateStereoCamsTransfromFromTopics-request> (roslisp-msg-protocol:ros-message)
  ((cam_1_pose_topic_name
    :reader cam_1_pose_topic_name
    :initarg :cam_1_pose_topic_name
    :type cl:string
    :initform "")
   (cam_2_pose_topic_name
    :reader cam_2_pose_topic_name
    :initarg :cam_2_pose_topic_name
    :type cl:string
    :initform ""))
)

(cl:defclass CalculateStereoCamsTransfromFromTopics-request (<CalculateStereoCamsTransfromFromTopics-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CalculateStereoCamsTransfromFromTopics-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CalculateStereoCamsTransfromFromTopics-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name teleop_vision-srv:<CalculateStereoCamsTransfromFromTopics-request> is deprecated: use teleop_vision-srv:CalculateStereoCamsTransfromFromTopics-request instead.")))

(cl:ensure-generic-function 'cam_1_pose_topic_name-val :lambda-list '(m))
(cl:defmethod cam_1_pose_topic_name-val ((m <CalculateStereoCamsTransfromFromTopics-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader teleop_vision-srv:cam_1_pose_topic_name-val is deprecated.  Use teleop_vision-srv:cam_1_pose_topic_name instead.")
  (cam_1_pose_topic_name m))

(cl:ensure-generic-function 'cam_2_pose_topic_name-val :lambda-list '(m))
(cl:defmethod cam_2_pose_topic_name-val ((m <CalculateStereoCamsTransfromFromTopics-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader teleop_vision-srv:cam_2_pose_topic_name-val is deprecated.  Use teleop_vision-srv:cam_2_pose_topic_name instead.")
  (cam_2_pose_topic_name m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CalculateStereoCamsTransfromFromTopics-request>) ostream)
  "Serializes a message object of type '<CalculateStereoCamsTransfromFromTopics-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'cam_1_pose_topic_name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'cam_1_pose_topic_name))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'cam_2_pose_topic_name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'cam_2_pose_topic_name))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CalculateStereoCamsTransfromFromTopics-request>) istream)
  "Deserializes a message object of type '<CalculateStereoCamsTransfromFromTopics-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'cam_1_pose_topic_name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'cam_1_pose_topic_name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'cam_2_pose_topic_name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'cam_2_pose_topic_name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CalculateStereoCamsTransfromFromTopics-request>)))
  "Returns string type for a service object of type '<CalculateStereoCamsTransfromFromTopics-request>"
  "teleop_vision/CalculateStereoCamsTransfromFromTopicsRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CalculateStereoCamsTransfromFromTopics-request)))
  "Returns string type for a service object of type 'CalculateStereoCamsTransfromFromTopics-request"
  "teleop_vision/CalculateStereoCamsTransfromFromTopicsRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CalculateStereoCamsTransfromFromTopics-request>)))
  "Returns md5sum for a message object of type '<CalculateStereoCamsTransfromFromTopics-request>"
  "28012a097e6af14c818b4f0effba0194")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CalculateStereoCamsTransfromFromTopics-request)))
  "Returns md5sum for a message object of type 'CalculateStereoCamsTransfromFromTopics-request"
  "28012a097e6af14c818b4f0effba0194")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CalculateStereoCamsTransfromFromTopics-request>)))
  "Returns full string definition for message of type '<CalculateStereoCamsTransfromFromTopics-request>"
  (cl:format cl:nil "string cam_1_pose_topic_name~%string cam_2_pose_topic_name~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CalculateStereoCamsTransfromFromTopics-request)))
  "Returns full string definition for message of type 'CalculateStereoCamsTransfromFromTopics-request"
  (cl:format cl:nil "string cam_1_pose_topic_name~%string cam_2_pose_topic_name~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CalculateStereoCamsTransfromFromTopics-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'cam_1_pose_topic_name))
     4 (cl:length (cl:slot-value msg 'cam_2_pose_topic_name))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CalculateStereoCamsTransfromFromTopics-request>))
  "Converts a ROS message object to a list"
  (cl:list 'CalculateStereoCamsTransfromFromTopics-request
    (cl:cons ':cam_1_pose_topic_name (cam_1_pose_topic_name msg))
    (cl:cons ':cam_2_pose_topic_name (cam_2_pose_topic_name msg))
))
;//! \htmlinclude CalculateStereoCamsTransfromFromTopics-response.msg.html

(cl:defclass <CalculateStereoCamsTransfromFromTopics-response> (roslisp-msg-protocol:ros-message)
  ((cam_1_to_cam_2_pose
    :reader cam_1_to_cam_2_pose
    :initarg :cam_1_to_cam_2_pose
    :type geometry_msgs-msg:Pose
    :initform (cl:make-instance 'geometry_msgs-msg:Pose)))
)

(cl:defclass CalculateStereoCamsTransfromFromTopics-response (<CalculateStereoCamsTransfromFromTopics-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CalculateStereoCamsTransfromFromTopics-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CalculateStereoCamsTransfromFromTopics-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name teleop_vision-srv:<CalculateStereoCamsTransfromFromTopics-response> is deprecated: use teleop_vision-srv:CalculateStereoCamsTransfromFromTopics-response instead.")))

(cl:ensure-generic-function 'cam_1_to_cam_2_pose-val :lambda-list '(m))
(cl:defmethod cam_1_to_cam_2_pose-val ((m <CalculateStereoCamsTransfromFromTopics-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader teleop_vision-srv:cam_1_to_cam_2_pose-val is deprecated.  Use teleop_vision-srv:cam_1_to_cam_2_pose instead.")
  (cam_1_to_cam_2_pose m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CalculateStereoCamsTransfromFromTopics-response>) ostream)
  "Serializes a message object of type '<CalculateStereoCamsTransfromFromTopics-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'cam_1_to_cam_2_pose) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CalculateStereoCamsTransfromFromTopics-response>) istream)
  "Deserializes a message object of type '<CalculateStereoCamsTransfromFromTopics-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'cam_1_to_cam_2_pose) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CalculateStereoCamsTransfromFromTopics-response>)))
  "Returns string type for a service object of type '<CalculateStereoCamsTransfromFromTopics-response>"
  "teleop_vision/CalculateStereoCamsTransfromFromTopicsResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CalculateStereoCamsTransfromFromTopics-response)))
  "Returns string type for a service object of type 'CalculateStereoCamsTransfromFromTopics-response"
  "teleop_vision/CalculateStereoCamsTransfromFromTopicsResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CalculateStereoCamsTransfromFromTopics-response>)))
  "Returns md5sum for a message object of type '<CalculateStereoCamsTransfromFromTopics-response>"
  "28012a097e6af14c818b4f0effba0194")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CalculateStereoCamsTransfromFromTopics-response)))
  "Returns md5sum for a message object of type 'CalculateStereoCamsTransfromFromTopics-response"
  "28012a097e6af14c818b4f0effba0194")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CalculateStereoCamsTransfromFromTopics-response>)))
  "Returns full string definition for message of type '<CalculateStereoCamsTransfromFromTopics-response>"
  (cl:format cl:nil "geometry_msgs/Pose cam_1_to_cam_2_pose~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CalculateStereoCamsTransfromFromTopics-response)))
  "Returns full string definition for message of type 'CalculateStereoCamsTransfromFromTopics-response"
  (cl:format cl:nil "geometry_msgs/Pose cam_1_to_cam_2_pose~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CalculateStereoCamsTransfromFromTopics-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'cam_1_to_cam_2_pose))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CalculateStereoCamsTransfromFromTopics-response>))
  "Converts a ROS message object to a list"
  (cl:list 'CalculateStereoCamsTransfromFromTopics-response
    (cl:cons ':cam_1_to_cam_2_pose (cam_1_to_cam_2_pose msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'CalculateStereoCamsTransfromFromTopics)))
  'CalculateStereoCamsTransfromFromTopics-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'CalculateStereoCamsTransfromFromTopics)))
  'CalculateStereoCamsTransfromFromTopics-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CalculateStereoCamsTransfromFromTopics)))
  "Returns string type for a service object of type '<CalculateStereoCamsTransfromFromTopics>"
  "teleop_vision/CalculateStereoCamsTransfromFromTopics")
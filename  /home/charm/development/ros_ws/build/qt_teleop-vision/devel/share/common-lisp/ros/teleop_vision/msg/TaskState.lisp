; Auto-generated. Do not edit!


(cl:in-package teleop_vision-msg)


;//! \htmlinclude TaskState.msg.html

(cl:defclass <TaskState> (roslisp-msg-protocol:ros-message)
  ((task_name
    :reader task_name
    :initarg :task_name
    :type cl:string
    :initform "")
   (task_state
    :reader task_state
    :initarg :task_state
    :type cl:fixnum
    :initform 0)
   (number_of_repetition
    :reader number_of_repetition
    :initarg :number_of_repetition
    :type cl:fixnum
    :initform 0)
   (time_stamp
    :reader time_stamp
    :initarg :time_stamp
    :type cl:float
    :initform 0.0)
   (position_error_norm
    :reader position_error_norm
    :initarg :position_error_norm
    :type cl:float
    :initform 0.0))
)

(cl:defclass TaskState (<TaskState>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TaskState>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TaskState)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name teleop_vision-msg:<TaskState> is deprecated: use teleop_vision-msg:TaskState instead.")))

(cl:ensure-generic-function 'task_name-val :lambda-list '(m))
(cl:defmethod task_name-val ((m <TaskState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader teleop_vision-msg:task_name-val is deprecated.  Use teleop_vision-msg:task_name instead.")
  (task_name m))

(cl:ensure-generic-function 'task_state-val :lambda-list '(m))
(cl:defmethod task_state-val ((m <TaskState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader teleop_vision-msg:task_state-val is deprecated.  Use teleop_vision-msg:task_state instead.")
  (task_state m))

(cl:ensure-generic-function 'number_of_repetition-val :lambda-list '(m))
(cl:defmethod number_of_repetition-val ((m <TaskState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader teleop_vision-msg:number_of_repetition-val is deprecated.  Use teleop_vision-msg:number_of_repetition instead.")
  (number_of_repetition m))

(cl:ensure-generic-function 'time_stamp-val :lambda-list '(m))
(cl:defmethod time_stamp-val ((m <TaskState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader teleop_vision-msg:time_stamp-val is deprecated.  Use teleop_vision-msg:time_stamp instead.")
  (time_stamp m))

(cl:ensure-generic-function 'position_error_norm-val :lambda-list '(m))
(cl:defmethod position_error_norm-val ((m <TaskState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader teleop_vision-msg:position_error_norm-val is deprecated.  Use teleop_vision-msg:position_error_norm instead.")
  (position_error_norm m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TaskState>) ostream)
  "Serializes a message object of type '<TaskState>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'task_name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'task_name))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'task_state)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'number_of_repetition)) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'time_stamp))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'position_error_norm))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TaskState>) istream)
  "Deserializes a message object of type '<TaskState>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'task_name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'task_name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'task_state)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'number_of_repetition)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'time_stamp) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'position_error_norm) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TaskState>)))
  "Returns string type for a message object of type '<TaskState>"
  "teleop_vision/TaskState")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TaskState)))
  "Returns string type for a message object of type 'TaskState"
  "teleop_vision/TaskState")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TaskState>)))
  "Returns md5sum for a message object of type '<TaskState>"
  "88680b1f4b0d4199c729e843287035e4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TaskState)))
  "Returns md5sum for a message object of type 'TaskState"
  "88680b1f4b0d4199c729e843287035e4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TaskState>)))
  "Returns full string definition for message of type '<TaskState>"
  (cl:format cl:nil "string task_name~%uint8 task_state~%uint8 number_of_repetition~%float64 time_stamp~%float64 position_error_norm~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TaskState)))
  "Returns full string definition for message of type 'TaskState"
  (cl:format cl:nil "string task_name~%uint8 task_state~%uint8 number_of_repetition~%float64 time_stamp~%float64 position_error_norm~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TaskState>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'task_name))
     1
     1
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TaskState>))
  "Converts a ROS message object to a list"
  (cl:list 'TaskState
    (cl:cons ':task_name (task_name msg))
    (cl:cons ':task_state (task_state msg))
    (cl:cons ':number_of_repetition (number_of_repetition msg))
    (cl:cons ':time_stamp (time_stamp msg))
    (cl:cons ':position_error_norm (position_error_norm msg))
))

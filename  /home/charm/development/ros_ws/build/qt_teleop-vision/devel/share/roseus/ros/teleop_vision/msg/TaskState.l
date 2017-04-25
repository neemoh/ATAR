;; Auto-generated. Do not edit!


(when (boundp 'teleop_vision::TaskState)
  (if (not (find-package "TELEOP_VISION"))
    (make-package "TELEOP_VISION"))
  (shadow 'TaskState (find-package "TELEOP_VISION")))
(unless (find-package "TELEOP_VISION::TASKSTATE")
  (make-package "TELEOP_VISION::TASKSTATE"))

(in-package "ROS")
;;//! \htmlinclude TaskState.msg.html


(defclass teleop_vision::TaskState
  :super ros::object
  :slots (_task_name _task_state _number_of_repetition _time_stamp _position_error_norm ))

(defmethod teleop_vision::TaskState
  (:init
   (&key
    ((:task_name __task_name) "")
    ((:task_state __task_state) 0)
    ((:number_of_repetition __number_of_repetition) 0)
    ((:time_stamp __time_stamp) 0.0)
    ((:position_error_norm __position_error_norm) 0.0)
    )
   (send-super :init)
   (setq _task_name (string __task_name))
   (setq _task_state (round __task_state))
   (setq _number_of_repetition (round __number_of_repetition))
   (setq _time_stamp (float __time_stamp))
   (setq _position_error_norm (float __position_error_norm))
   self)
  (:task_name
   (&optional __task_name)
   (if __task_name (setq _task_name __task_name)) _task_name)
  (:task_state
   (&optional __task_state)
   (if __task_state (setq _task_state __task_state)) _task_state)
  (:number_of_repetition
   (&optional __number_of_repetition)
   (if __number_of_repetition (setq _number_of_repetition __number_of_repetition)) _number_of_repetition)
  (:time_stamp
   (&optional __time_stamp)
   (if __time_stamp (setq _time_stamp __time_stamp)) _time_stamp)
  (:position_error_norm
   (&optional __position_error_norm)
   (if __position_error_norm (setq _position_error_norm __position_error_norm)) _position_error_norm)
  (:serialization-length
   ()
   (+
    ;; string _task_name
    4 (length _task_name)
    ;; uint8 _task_state
    1
    ;; uint8 _number_of_repetition
    1
    ;; float64 _time_stamp
    8
    ;; float64 _position_error_norm
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _task_name
       (write-long (length _task_name) s) (princ _task_name s)
     ;; uint8 _task_state
       (write-byte _task_state s)
     ;; uint8 _number_of_repetition
       (write-byte _number_of_repetition s)
     ;; float64 _time_stamp
       (sys::poke _time_stamp (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _position_error_norm
       (sys::poke _position_error_norm (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _task_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _task_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; uint8 _task_state
     (setq _task_state (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _number_of_repetition
     (setq _number_of_repetition (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; float64 _time_stamp
     (setq _time_stamp (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _position_error_norm
     (setq _position_error_norm (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get teleop_vision::TaskState :md5sum-) "88680b1f4b0d4199c729e843287035e4")
(setf (get teleop_vision::TaskState :datatype-) "teleop_vision/TaskState")
(setf (get teleop_vision::TaskState :definition-)
      "string task_name
uint8 task_state
uint8 number_of_repetition
float64 time_stamp
float64 position_error_norm
")



(provide :teleop_vision/TaskState "88680b1f4b0d4199c729e843287035e4")



;; Auto-generated. Do not edit!


(when (boundp 'custom_msg_week3::custom)
  (if (not (find-package "CUSTOM_MSG_WEEK3"))
    (make-package "CUSTOM_MSG_WEEK3"))
  (shadow 'custom (find-package "CUSTOM_MSG_WEEK3")))
(unless (find-package "CUSTOM_MSG_WEEK3::CUSTOM")
  (make-package "CUSTOM_MSG_WEEK3::CUSTOM"))

(in-package "ROS")
;;//! \htmlinclude custom.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass custom_msg_week3::custom
  :super ros::object
  :slots (_user_input ))

(defmethod custom_msg_week3::custom
  (:init
   (&key
    ((:user_input __user_input) (instance std_msgs::String :init))
    )
   (send-super :init)
   (setq _user_input __user_input)
   self)
  (:user_input
   (&rest __user_input)
   (if (keywordp (car __user_input))
       (send* _user_input __user_input)
     (progn
       (if __user_input (setq _user_input (car __user_input)))
       _user_input)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/String _user_input
    (send _user_input :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/String _user_input
       (send _user_input :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/String _user_input
     (send _user_input :deserialize buf ptr-) (incf ptr- (send _user_input :serialization-length))
   ;;
   self)
  )

(setf (get custom_msg_week3::custom :md5sum-) "f3d8e52418e879f2a35002f345d5e6fb")
(setf (get custom_msg_week3::custom :datatype-) "custom_msg_week3/custom")
(setf (get custom_msg_week3::custom :definition-)
      "std_msgs/String user_input
================================================================================
MSG: std_msgs/String
string data

")



(provide :custom_msg_week3/custom "f3d8e52418e879f2a35002f345d5e6fb")



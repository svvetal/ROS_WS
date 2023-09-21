; Auto-generated. Do not edit!


(cl:in-package custom_msg_week3-msg)


;//! \htmlinclude custom.msg.html

(cl:defclass <custom> (roslisp-msg-protocol:ros-message)
  ((user_input
    :reader user_input
    :initarg :user_input
    :type std_msgs-msg:String
    :initform (cl:make-instance 'std_msgs-msg:String)))
)

(cl:defclass custom (<custom>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <custom>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'custom)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name custom_msg_week3-msg:<custom> is deprecated: use custom_msg_week3-msg:custom instead.")))

(cl:ensure-generic-function 'user_input-val :lambda-list '(m))
(cl:defmethod user_input-val ((m <custom>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader custom_msg_week3-msg:user_input-val is deprecated.  Use custom_msg_week3-msg:user_input instead.")
  (user_input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <custom>) ostream)
  "Serializes a message object of type '<custom>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'user_input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <custom>) istream)
  "Deserializes a message object of type '<custom>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'user_input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<custom>)))
  "Returns string type for a message object of type '<custom>"
  "custom_msg_week3/custom")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'custom)))
  "Returns string type for a message object of type 'custom"
  "custom_msg_week3/custom")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<custom>)))
  "Returns md5sum for a message object of type '<custom>"
  "f3d8e52418e879f2a35002f345d5e6fb")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'custom)))
  "Returns md5sum for a message object of type 'custom"
  "f3d8e52418e879f2a35002f345d5e6fb")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<custom>)))
  "Returns full string definition for message of type '<custom>"
  (cl:format cl:nil "std_msgs/String user_input~%================================================================================~%MSG: std_msgs/String~%string data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'custom)))
  "Returns full string definition for message of type 'custom"
  (cl:format cl:nil "std_msgs/String user_input~%================================================================================~%MSG: std_msgs/String~%string data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <custom>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'user_input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <custom>))
  "Converts a ROS message object to a list"
  (cl:list 'custom
    (cl:cons ':user_input (user_input msg))
))

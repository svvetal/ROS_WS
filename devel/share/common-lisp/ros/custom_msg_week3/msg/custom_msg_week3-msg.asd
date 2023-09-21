
(cl:in-package :asdf)

(defsystem "custom_msg_week3-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "custom" :depends-on ("_package_custom"))
    (:file "_package_custom" :depends-on ("_package"))
  ))
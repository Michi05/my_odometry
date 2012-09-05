
(cl:in-package :asdf)

(defsystem "my_odometry-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :nav_msgs-msg
               :tf2_msgs-msg
)
  :components ((:file "_package")
    (:file "odom_answer" :depends-on ("_package_odom_answer"))
    (:file "_package_odom_answer" :depends-on ("_package"))
  ))
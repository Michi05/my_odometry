
(cl:in-package :asdf)

(defsystem "my_odometry-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :my_odometry-msg
)
  :components ((:file "_package")
    (:file "emptyRequest" :depends-on ("_package_emptyRequest"))
    (:file "_package_emptyRequest" :depends-on ("_package"))
    (:file "statusMsg" :depends-on ("_package_statusMsg"))
    (:file "_package_statusMsg" :depends-on ("_package"))
    (:file "odom_update_srv" :depends-on ("_package_odom_update_srv"))
    (:file "_package_odom_update_srv" :depends-on ("_package"))
  ))
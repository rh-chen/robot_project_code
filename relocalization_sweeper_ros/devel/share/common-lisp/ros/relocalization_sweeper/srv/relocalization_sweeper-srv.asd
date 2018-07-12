
(cl:in-package :asdf)

(defsystem "relocalization_sweeper-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :sensor_msgs-msg
)
  :components ((:file "_package")
    (:file "GetRobotPose" :depends-on ("_package_GetRobotPose"))
    (:file "_package_GetRobotPose" :depends-on ("_package"))
  ))
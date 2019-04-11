
(cl:in-package :asdf)

(defsystem "coverage_path_planning-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :nav_msgs-msg
)
  :components ((:file "_package")
    (:file "GetCoveragePath" :depends-on ("_package_GetCoveragePath"))
    (:file "_package_GetCoveragePath" :depends-on ("_package"))
  ))
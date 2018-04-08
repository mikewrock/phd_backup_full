
(cl:in-package :asdf)

(defsystem "phd-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :phd-msg
               :sensor_msgs-msg
)
  :components ((:file "_package")
    (:file "thickness_service" :depends-on ("_package_thickness_service"))
    (:file "_package_thickness_service" :depends-on ("_package"))
    (:file "localize_cloud" :depends-on ("_package_localize_cloud"))
    (:file "_package_localize_cloud" :depends-on ("_package"))
    (:file "trajectory_service (nonlinear)" :depends-on ("_package_trajectory_service (nonlinear)"))
    (:file "_package_trajectory_service (nonlinear)" :depends-on ("_package"))
    (:file "trajectory_service" :depends-on ("_package_trajectory_service"))
    (:file "_package_trajectory_service" :depends-on ("_package"))
    (:file "simple_trajectory_service" :depends-on ("_package_simple_trajectory_service"))
    (:file "_package_simple_trajectory_service" :depends-on ("_package"))
    (:file "empty" :depends-on ("_package_empty"))
    (:file "_package_empty" :depends-on ("_package"))
  ))
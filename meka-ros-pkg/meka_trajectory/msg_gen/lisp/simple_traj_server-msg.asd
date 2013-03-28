
(cl:in-package :asdf)

(defsystem "simple_traj_server-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :actionlib_msgs-msg
               :std_msgs-msg
               :trajectory_msgs-msg
)
  :components ((:file "_package")
    (:file "TrajGoal" :depends-on ("_package_TrajGoal"))
    (:file "_package_TrajGoal" :depends-on ("_package"))
    (:file "TrajActionResult" :depends-on ("_package_TrajActionResult"))
    (:file "_package_TrajActionResult" :depends-on ("_package"))
    (:file "TrajFeedback" :depends-on ("_package_TrajFeedback"))
    (:file "_package_TrajFeedback" :depends-on ("_package"))
    (:file "TrajAction" :depends-on ("_package_TrajAction"))
    (:file "_package_TrajAction" :depends-on ("_package"))
    (:file "TrajResult" :depends-on ("_package_TrajResult"))
    (:file "_package_TrajResult" :depends-on ("_package"))
    (:file "TrajActionGoal" :depends-on ("_package_TrajActionGoal"))
    (:file "_package_TrajActionGoal" :depends-on ("_package"))
    (:file "TrajActionFeedback" :depends-on ("_package_TrajActionFeedback"))
    (:file "_package_TrajActionFeedback" :depends-on ("_package"))
  ))
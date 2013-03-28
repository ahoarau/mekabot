
(cl:in-package :asdf)

(defsystem "m3ctrl_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "M3JointCmd" :depends-on ("_package_M3JointCmd"))
    (:file "_package_M3JointCmd" :depends-on ("_package"))
  ))
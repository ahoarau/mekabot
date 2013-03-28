
(cl:in-package :asdf)

(defsystem "m3meka_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :m3_msgs-msg
)
  :components ((:file "_package")
    (:file "M3JointStatus" :depends-on ("_package_M3JointStatus"))
    (:file "_package_M3JointStatus" :depends-on ("_package"))
    (:file "M3OmnibaseJoy" :depends-on ("_package_M3OmnibaseJoy"))
    (:file "_package_M3OmnibaseJoy" :depends-on ("_package"))
    (:file "M3JointCmd" :depends-on ("_package_M3JointCmd"))
    (:file "_package_M3JointCmd" :depends-on ("_package"))
  ))
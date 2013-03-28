
(in-package :asdf)

(defsystem "m3_client-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils)
  :components ((:file "_package")
    (:file "M3OmnibaseJoy" :depends-on ("_package"))
    (:file "_package_M3OmnibaseJoy" :depends-on ("_package"))
    (:file "M3BaseStatus" :depends-on ("_package"))
    (:file "_package_M3BaseStatus" :depends-on ("_package"))
    ))

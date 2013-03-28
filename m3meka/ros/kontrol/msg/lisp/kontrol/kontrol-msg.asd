
(in-package :asdf)

(defsystem "kontrol-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils)
  :components ((:file "_package")
    (:file "Kontrol" :depends-on ("_package"))
    (:file "_package_Kontrol" :depends-on ("_package"))
    ))


(cl:in-package :asdf)

(defsystem "kontrol-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Kontrol" :depends-on ("_package_Kontrol"))
    (:file "_package_Kontrol" :depends-on ("_package"))
  ))
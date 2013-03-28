
(cl:in-package :asdf)

(defsystem "mic_array-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "MicArray" :depends-on ("_package_MicArray"))
    (:file "_package_MicArray" :depends-on ("_package"))
  ))
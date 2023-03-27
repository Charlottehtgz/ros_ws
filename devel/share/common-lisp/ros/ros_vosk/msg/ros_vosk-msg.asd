
(cl:in-package :asdf)

(defsystem "ros_vosk-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "speech_recognition" :depends-on ("_package_speech_recognition"))
    (:file "_package_speech_recognition" :depends-on ("_package"))
  ))
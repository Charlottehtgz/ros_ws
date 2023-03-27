;; Auto-generated. Do not edit!


(when (boundp 'ros_vosk::speech_recognition)
  (if (not (find-package "ROS_VOSK"))
    (make-package "ROS_VOSK"))
  (shadow 'speech_recognition (find-package "ROS_VOSK")))
(unless (find-package "ROS_VOSK::SPEECH_RECOGNITION")
  (make-package "ROS_VOSK::SPEECH_RECOGNITION"))

(in-package "ROS")
;;//! \htmlinclude speech_recognition.msg.html


(defclass ros_vosk::speech_recognition
  :super ros::object
  :slots (_time_recognized _final_result _partial_result _isSpeech_recognized ))

(defmethod ros_vosk::speech_recognition
  (:init
   (&key
    ((:time_recognized __time_recognized) (instance ros::time :init))
    ((:final_result __final_result) "")
    ((:partial_result __partial_result) "")
    ((:isSpeech_recognized __isSpeech_recognized) nil)
    )
   (send-super :init)
   (setq _time_recognized __time_recognized)
   (setq _final_result (string __final_result))
   (setq _partial_result (string __partial_result))
   (setq _isSpeech_recognized __isSpeech_recognized)
   self)
  (:time_recognized
   (&optional __time_recognized)
   (if __time_recognized (setq _time_recognized __time_recognized)) _time_recognized)
  (:final_result
   (&optional __final_result)
   (if __final_result (setq _final_result __final_result)) _final_result)
  (:partial_result
   (&optional __partial_result)
   (if __partial_result (setq _partial_result __partial_result)) _partial_result)
  (:isSpeech_recognized
   (&optional (__isSpeech_recognized :null))
   (if (not (eq __isSpeech_recognized :null)) (setq _isSpeech_recognized __isSpeech_recognized)) _isSpeech_recognized)
  (:serialization-length
   ()
   (+
    ;; time _time_recognized
    8
    ;; string _final_result
    4 (length _final_result)
    ;; string _partial_result
    4 (length _partial_result)
    ;; bool _isSpeech_recognized
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; time _time_recognized
       (write-long (send _time_recognized :sec) s) (write-long (send _time_recognized :nsec) s)
     ;; string _final_result
       (write-long (length _final_result) s) (princ _final_result s)
     ;; string _partial_result
       (write-long (length _partial_result) s) (princ _partial_result s)
     ;; bool _isSpeech_recognized
       (if _isSpeech_recognized (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; time _time_recognized
     (send _time_recognized :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _time_recognized :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; string _final_result
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _final_result (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _partial_result
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _partial_result (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; bool _isSpeech_recognized
     (setq _isSpeech_recognized (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get ros_vosk::speech_recognition :md5sum-) "d3b0b667a7ab9370dea4b81b6e8de6c9")
(setf (get ros_vosk::speech_recognition :datatype-) "ros_vosk/speech_recognition")
(setf (get ros_vosk::speech_recognition :definition-)
      "time    time_recognized
string  final_result
string  partial_result 
bool    isSpeech_recognized
")



(provide :ros_vosk/speech_recognition "d3b0b667a7ab9370dea4b81b6e8de6c9")



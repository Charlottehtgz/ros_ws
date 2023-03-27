; Auto-generated. Do not edit!


(cl:in-package ros_vosk-msg)


;//! \htmlinclude speech_recognition.msg.html

(cl:defclass <speech_recognition> (roslisp-msg-protocol:ros-message)
  ((time_recognized
    :reader time_recognized
    :initarg :time_recognized
    :type cl:real
    :initform 0)
   (final_result
    :reader final_result
    :initarg :final_result
    :type cl:string
    :initform "")
   (partial_result
    :reader partial_result
    :initarg :partial_result
    :type cl:string
    :initform "")
   (isSpeech_recognized
    :reader isSpeech_recognized
    :initarg :isSpeech_recognized
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass speech_recognition (<speech_recognition>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <speech_recognition>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'speech_recognition)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ros_vosk-msg:<speech_recognition> is deprecated: use ros_vosk-msg:speech_recognition instead.")))

(cl:ensure-generic-function 'time_recognized-val :lambda-list '(m))
(cl:defmethod time_recognized-val ((m <speech_recognition>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_vosk-msg:time_recognized-val is deprecated.  Use ros_vosk-msg:time_recognized instead.")
  (time_recognized m))

(cl:ensure-generic-function 'final_result-val :lambda-list '(m))
(cl:defmethod final_result-val ((m <speech_recognition>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_vosk-msg:final_result-val is deprecated.  Use ros_vosk-msg:final_result instead.")
  (final_result m))

(cl:ensure-generic-function 'partial_result-val :lambda-list '(m))
(cl:defmethod partial_result-val ((m <speech_recognition>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_vosk-msg:partial_result-val is deprecated.  Use ros_vosk-msg:partial_result instead.")
  (partial_result m))

(cl:ensure-generic-function 'isSpeech_recognized-val :lambda-list '(m))
(cl:defmethod isSpeech_recognized-val ((m <speech_recognition>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_vosk-msg:isSpeech_recognized-val is deprecated.  Use ros_vosk-msg:isSpeech_recognized instead.")
  (isSpeech_recognized m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <speech_recognition>) ostream)
  "Serializes a message object of type '<speech_recognition>"
  (cl:let ((__sec (cl:floor (cl:slot-value msg 'time_recognized)))
        (__nsec (cl:round (cl:* 1e9 (cl:- (cl:slot-value msg 'time_recognized) (cl:floor (cl:slot-value msg 'time_recognized)))))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 0) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __nsec) ostream))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'final_result))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'final_result))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'partial_result))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'partial_result))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'isSpeech_recognized) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <speech_recognition>) istream)
  "Deserializes a message object of type '<speech_recognition>"
    (cl:let ((__sec 0) (__nsec 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 0) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __nsec) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'time_recognized) (cl:+ (cl:coerce __sec 'cl:double-float) (cl:/ __nsec 1e9))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'final_result) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'final_result) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'partial_result) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'partial_result) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:slot-value msg 'isSpeech_recognized) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<speech_recognition>)))
  "Returns string type for a message object of type '<speech_recognition>"
  "ros_vosk/speech_recognition")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'speech_recognition)))
  "Returns string type for a message object of type 'speech_recognition"
  "ros_vosk/speech_recognition")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<speech_recognition>)))
  "Returns md5sum for a message object of type '<speech_recognition>"
  "d3b0b667a7ab9370dea4b81b6e8de6c9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'speech_recognition)))
  "Returns md5sum for a message object of type 'speech_recognition"
  "d3b0b667a7ab9370dea4b81b6e8de6c9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<speech_recognition>)))
  "Returns full string definition for message of type '<speech_recognition>"
  (cl:format cl:nil "time    time_recognized~%string  final_result~%string  partial_result ~%bool    isSpeech_recognized~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'speech_recognition)))
  "Returns full string definition for message of type 'speech_recognition"
  (cl:format cl:nil "time    time_recognized~%string  final_result~%string  partial_result ~%bool    isSpeech_recognized~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <speech_recognition>))
  (cl:+ 0
     8
     4 (cl:length (cl:slot-value msg 'final_result))
     4 (cl:length (cl:slot-value msg 'partial_result))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <speech_recognition>))
  "Converts a ROS message object to a list"
  (cl:list 'speech_recognition
    (cl:cons ':time_recognized (time_recognized msg))
    (cl:cons ':final_result (final_result msg))
    (cl:cons ':partial_result (partial_result msg))
    (cl:cons ':isSpeech_recognized (isSpeech_recognized msg))
))

; Auto-generated. Do not edit!


(cl:in-package mic_array-srv)


;//! \htmlinclude MicArrayParam-request.msg.html

(cl:defclass <MicArrayParam-request> (roslisp-msg-protocol:ros-message)
  ((gains
    :reader gains
    :initarg :gains
    :type (cl:vector cl:float)
   :initform (cl:make-array 6 :element-type 'cl:float :initial-element 0.0))
   (bias
    :reader bias
    :initarg :bias
    :type (cl:vector cl:float)
   :initform (cl:make-array 6 :element-type 'cl:float :initial-element 0.0))
   (window_time
    :reader window_time
    :initarg :window_time
    :type cl:float
    :initform 0.0)
   (threshold
    :reader threshold
    :initarg :threshold
    :type cl:float
    :initform 0.0)
   (slew_rate
    :reader slew_rate
    :initarg :slew_rate
    :type cl:float
    :initform 0.0))
)

(cl:defclass MicArrayParam-request (<MicArrayParam-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MicArrayParam-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MicArrayParam-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mic_array-srv:<MicArrayParam-request> is deprecated: use mic_array-srv:MicArrayParam-request instead.")))

(cl:ensure-generic-function 'gains-val :lambda-list '(m))
(cl:defmethod gains-val ((m <MicArrayParam-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mic_array-srv:gains-val is deprecated.  Use mic_array-srv:gains instead.")
  (gains m))

(cl:ensure-generic-function 'bias-val :lambda-list '(m))
(cl:defmethod bias-val ((m <MicArrayParam-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mic_array-srv:bias-val is deprecated.  Use mic_array-srv:bias instead.")
  (bias m))

(cl:ensure-generic-function 'window_time-val :lambda-list '(m))
(cl:defmethod window_time-val ((m <MicArrayParam-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mic_array-srv:window_time-val is deprecated.  Use mic_array-srv:window_time instead.")
  (window_time m))

(cl:ensure-generic-function 'threshold-val :lambda-list '(m))
(cl:defmethod threshold-val ((m <MicArrayParam-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mic_array-srv:threshold-val is deprecated.  Use mic_array-srv:threshold instead.")
  (threshold m))

(cl:ensure-generic-function 'slew_rate-val :lambda-list '(m))
(cl:defmethod slew_rate-val ((m <MicArrayParam-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mic_array-srv:slew_rate-val is deprecated.  Use mic_array-srv:slew_rate instead.")
  (slew_rate m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MicArrayParam-request>) ostream)
  "Serializes a message object of type '<MicArrayParam-request>"
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'gains))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'bias))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'window_time))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'threshold))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'slew_rate))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MicArrayParam-request>) istream)
  "Deserializes a message object of type '<MicArrayParam-request>"
  (cl:setf (cl:slot-value msg 'gains) (cl:make-array 6))
  (cl:let ((vals (cl:slot-value msg 'gains)))
    (cl:dotimes (i 6)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  (cl:setf (cl:slot-value msg 'bias) (cl:make-array 6))
  (cl:let ((vals (cl:slot-value msg 'bias)))
    (cl:dotimes (i 6)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'window_time) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'threshold) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'slew_rate) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MicArrayParam-request>)))
  "Returns string type for a service object of type '<MicArrayParam-request>"
  "mic_array/MicArrayParamRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MicArrayParam-request)))
  "Returns string type for a service object of type 'MicArrayParam-request"
  "mic_array/MicArrayParamRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MicArrayParam-request>)))
  "Returns md5sum for a message object of type '<MicArrayParam-request>"
  "5ac40a290a3cfd8edd9349aa1ca0043f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MicArrayParam-request)))
  "Returns md5sum for a message object of type 'MicArrayParam-request"
  "5ac40a290a3cfd8edd9349aa1ca0043f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MicArrayParam-request>)))
  "Returns full string definition for message of type '<MicArrayParam-request>"
  (cl:format cl:nil "float32[6] gains~%float32[6] bias~%float32 window_time~%float32 threshold~%float32 slew_rate~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MicArrayParam-request)))
  "Returns full string definition for message of type 'MicArrayParam-request"
  (cl:format cl:nil "float32[6] gains~%float32[6] bias~%float32 window_time~%float32 threshold~%float32 slew_rate~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MicArrayParam-request>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'gains) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'bias) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MicArrayParam-request>))
  "Converts a ROS message object to a list"
  (cl:list 'MicArrayParam-request
    (cl:cons ':gains (gains msg))
    (cl:cons ':bias (bias msg))
    (cl:cons ':window_time (window_time msg))
    (cl:cons ':threshold (threshold msg))
    (cl:cons ':slew_rate (slew_rate msg))
))
;//! \htmlinclude MicArrayParam-response.msg.html

(cl:defclass <MicArrayParam-response> (roslisp-msg-protocol:ros-message)
  ((resp
    :reader resp
    :initarg :resp
    :type cl:integer
    :initform 0))
)

(cl:defclass MicArrayParam-response (<MicArrayParam-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MicArrayParam-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MicArrayParam-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mic_array-srv:<MicArrayParam-response> is deprecated: use mic_array-srv:MicArrayParam-response instead.")))

(cl:ensure-generic-function 'resp-val :lambda-list '(m))
(cl:defmethod resp-val ((m <MicArrayParam-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mic_array-srv:resp-val is deprecated.  Use mic_array-srv:resp instead.")
  (resp m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MicArrayParam-response>) ostream)
  "Serializes a message object of type '<MicArrayParam-response>"
  (cl:let* ((signed (cl:slot-value msg 'resp)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MicArrayParam-response>) istream)
  "Deserializes a message object of type '<MicArrayParam-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'resp) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MicArrayParam-response>)))
  "Returns string type for a service object of type '<MicArrayParam-response>"
  "mic_array/MicArrayParamResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MicArrayParam-response)))
  "Returns string type for a service object of type 'MicArrayParam-response"
  "mic_array/MicArrayParamResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MicArrayParam-response>)))
  "Returns md5sum for a message object of type '<MicArrayParam-response>"
  "5ac40a290a3cfd8edd9349aa1ca0043f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MicArrayParam-response)))
  "Returns md5sum for a message object of type 'MicArrayParam-response"
  "5ac40a290a3cfd8edd9349aa1ca0043f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MicArrayParam-response>)))
  "Returns full string definition for message of type '<MicArrayParam-response>"
  (cl:format cl:nil "int32 resp~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MicArrayParam-response)))
  "Returns full string definition for message of type 'MicArrayParam-response"
  (cl:format cl:nil "int32 resp~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MicArrayParam-response>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MicArrayParam-response>))
  "Converts a ROS message object to a list"
  (cl:list 'MicArrayParam-response
    (cl:cons ':resp (resp msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'MicArrayParam)))
  'MicArrayParam-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'MicArrayParam)))
  'MicArrayParam-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MicArrayParam)))
  "Returns string type for a service object of type '<MicArrayParam>"
  "mic_array/MicArrayParam")
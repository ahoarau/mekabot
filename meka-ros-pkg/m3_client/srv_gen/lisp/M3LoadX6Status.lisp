; Auto-generated. Do not edit!


(cl:in-package m3_client-srv)


;//! \htmlinclude M3LoadX6Status-request.msg.html

(cl:defclass <M3LoadX6Status-request> (roslisp-msg-protocol:ros-message)
  ((request
    :reader request
    :initarg :request
    :type cl:integer
    :initform 0))
)

(cl:defclass M3LoadX6Status-request (<M3LoadX6Status-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <M3LoadX6Status-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'M3LoadX6Status-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name m3_client-srv:<M3LoadX6Status-request> is deprecated: use m3_client-srv:M3LoadX6Status-request instead.")))

(cl:ensure-generic-function 'request-val :lambda-list '(m))
(cl:defmethod request-val ((m <M3LoadX6Status-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader m3_client-srv:request-val is deprecated.  Use m3_client-srv:request instead.")
  (request m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <M3LoadX6Status-request>) ostream)
  "Serializes a message object of type '<M3LoadX6Status-request>"
  (cl:let* ((signed (cl:slot-value msg 'request)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <M3LoadX6Status-request>) istream)
  "Deserializes a message object of type '<M3LoadX6Status-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'request) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<M3LoadX6Status-request>)))
  "Returns string type for a service object of type '<M3LoadX6Status-request>"
  "m3_client/M3LoadX6StatusRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'M3LoadX6Status-request)))
  "Returns string type for a service object of type 'M3LoadX6Status-request"
  "m3_client/M3LoadX6StatusRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<M3LoadX6Status-request>)))
  "Returns md5sum for a message object of type '<M3LoadX6Status-request>"
  "967558453e196f040b12e56469a00597")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'M3LoadX6Status-request)))
  "Returns md5sum for a message object of type 'M3LoadX6Status-request"
  "967558453e196f040b12e56469a00597")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<M3LoadX6Status-request>)))
  "Returns full string definition for message of type '<M3LoadX6Status-request>"
  (cl:format cl:nil "int32 request~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'M3LoadX6Status-request)))
  "Returns full string definition for message of type 'M3LoadX6Status-request"
  (cl:format cl:nil "int32 request~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <M3LoadX6Status-request>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <M3LoadX6Status-request>))
  "Converts a ROS message object to a list"
  (cl:list 'M3LoadX6Status-request
    (cl:cons ':request (request msg))
))
;//! \htmlinclude M3LoadX6Status-response.msg.html

(cl:defclass <M3LoadX6Status-response> (roslisp-msg-protocol:ros-message)
  ((base
    :reader base
    :initarg :base
    :type m3_client-msg:M3BaseStatus
    :initform (cl:make-instance 'm3_client-msg:M3BaseStatus))
   (wrench
    :reader wrench
    :initarg :wrench
    :type (cl:vector cl:float)
   :initform (cl:make-array 6 :element-type 'cl:float :initial-element 0.0))
   (adc_ext_0
    :reader adc_ext_0
    :initarg :adc_ext_0
    :type cl:float
    :initform 0.0)
   (adc_ext_1
    :reader adc_ext_1
    :initarg :adc_ext_1
    :type cl:float
    :initform 0.0)
   (adc_ext_2
    :reader adc_ext_2
    :initarg :adc_ext_2
    :type cl:float
    :initform 0.0)
   (dig_ext_0
    :reader dig_ext_0
    :initarg :dig_ext_0
    :type cl:float
    :initform 0.0))
)

(cl:defclass M3LoadX6Status-response (<M3LoadX6Status-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <M3LoadX6Status-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'M3LoadX6Status-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name m3_client-srv:<M3LoadX6Status-response> is deprecated: use m3_client-srv:M3LoadX6Status-response instead.")))

(cl:ensure-generic-function 'base-val :lambda-list '(m))
(cl:defmethod base-val ((m <M3LoadX6Status-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader m3_client-srv:base-val is deprecated.  Use m3_client-srv:base instead.")
  (base m))

(cl:ensure-generic-function 'wrench-val :lambda-list '(m))
(cl:defmethod wrench-val ((m <M3LoadX6Status-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader m3_client-srv:wrench-val is deprecated.  Use m3_client-srv:wrench instead.")
  (wrench m))

(cl:ensure-generic-function 'adc_ext_0-val :lambda-list '(m))
(cl:defmethod adc_ext_0-val ((m <M3LoadX6Status-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader m3_client-srv:adc_ext_0-val is deprecated.  Use m3_client-srv:adc_ext_0 instead.")
  (adc_ext_0 m))

(cl:ensure-generic-function 'adc_ext_1-val :lambda-list '(m))
(cl:defmethod adc_ext_1-val ((m <M3LoadX6Status-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader m3_client-srv:adc_ext_1-val is deprecated.  Use m3_client-srv:adc_ext_1 instead.")
  (adc_ext_1 m))

(cl:ensure-generic-function 'adc_ext_2-val :lambda-list '(m))
(cl:defmethod adc_ext_2-val ((m <M3LoadX6Status-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader m3_client-srv:adc_ext_2-val is deprecated.  Use m3_client-srv:adc_ext_2 instead.")
  (adc_ext_2 m))

(cl:ensure-generic-function 'dig_ext_0-val :lambda-list '(m))
(cl:defmethod dig_ext_0-val ((m <M3LoadX6Status-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader m3_client-srv:dig_ext_0-val is deprecated.  Use m3_client-srv:dig_ext_0 instead.")
  (dig_ext_0 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <M3LoadX6Status-response>) ostream)
  "Serializes a message object of type '<M3LoadX6Status-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'base) ostream)
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'wrench))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'adc_ext_0))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'adc_ext_1))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'adc_ext_2))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'dig_ext_0))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <M3LoadX6Status-response>) istream)
  "Deserializes a message object of type '<M3LoadX6Status-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'base) istream)
  (cl:setf (cl:slot-value msg 'wrench) (cl:make-array 6))
  (cl:let ((vals (cl:slot-value msg 'wrench)))
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
    (cl:setf (cl:slot-value msg 'adc_ext_0) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'adc_ext_1) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'adc_ext_2) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'dig_ext_0) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<M3LoadX6Status-response>)))
  "Returns string type for a service object of type '<M3LoadX6Status-response>"
  "m3_client/M3LoadX6StatusResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'M3LoadX6Status-response)))
  "Returns string type for a service object of type 'M3LoadX6Status-response"
  "m3_client/M3LoadX6StatusResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<M3LoadX6Status-response>)))
  "Returns md5sum for a message object of type '<M3LoadX6Status-response>"
  "967558453e196f040b12e56469a00597")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'M3LoadX6Status-response)))
  "Returns md5sum for a message object of type 'M3LoadX6Status-response"
  "967558453e196f040b12e56469a00597")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<M3LoadX6Status-response>)))
  "Returns full string definition for message of type '<M3LoadX6Status-response>"
  (cl:format cl:nil "M3BaseStatus base~%float32[6] wrench~%float32 adc_ext_0~%float32 adc_ext_1~%float32 adc_ext_2~%float32 dig_ext_0~%~%~%================================================================================~%MSG: m3_client/M3BaseStatus~%string name~%uint8 state~%int64 timestamp~%string rate~%string version~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'M3LoadX6Status-response)))
  "Returns full string definition for message of type 'M3LoadX6Status-response"
  (cl:format cl:nil "M3BaseStatus base~%float32[6] wrench~%float32 adc_ext_0~%float32 adc_ext_1~%float32 adc_ext_2~%float32 dig_ext_0~%~%~%================================================================================~%MSG: m3_client/M3BaseStatus~%string name~%uint8 state~%int64 timestamp~%string rate~%string version~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <M3LoadX6Status-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'base))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'wrench) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <M3LoadX6Status-response>))
  "Converts a ROS message object to a list"
  (cl:list 'M3LoadX6Status-response
    (cl:cons ':base (base msg))
    (cl:cons ':wrench (wrench msg))
    (cl:cons ':adc_ext_0 (adc_ext_0 msg))
    (cl:cons ':adc_ext_1 (adc_ext_1 msg))
    (cl:cons ':adc_ext_2 (adc_ext_2 msg))
    (cl:cons ':dig_ext_0 (dig_ext_0 msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'M3LoadX6Status)))
  'M3LoadX6Status-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'M3LoadX6Status)))
  'M3LoadX6Status-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'M3LoadX6Status)))
  "Returns string type for a service object of type '<M3LoadX6Status>"
  "m3_client/M3LoadX6Status")
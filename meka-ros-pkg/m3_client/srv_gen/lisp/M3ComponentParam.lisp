; Auto-generated. Do not edit!


(cl:in-package m3_client-srv)


;//! \htmlinclude M3ComponentParam-request.msg.html

(cl:defclass <M3ComponentParam-request> (roslisp-msg-protocol:ros-message)
  ((a
    :reader a
    :initarg :a
    :type cl:integer
    :initform 0))
)

(cl:defclass M3ComponentParam-request (<M3ComponentParam-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <M3ComponentParam-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'M3ComponentParam-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name m3_client-srv:<M3ComponentParam-request> is deprecated: use m3_client-srv:M3ComponentParam-request instead.")))

(cl:ensure-generic-function 'a-val :lambda-list '(m))
(cl:defmethod a-val ((m <M3ComponentParam-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader m3_client-srv:a-val is deprecated.  Use m3_client-srv:a instead.")
  (a m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <M3ComponentParam-request>) ostream)
  "Serializes a message object of type '<M3ComponentParam-request>"
  (cl:let* ((signed (cl:slot-value msg 'a)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <M3ComponentParam-request>) istream)
  "Deserializes a message object of type '<M3ComponentParam-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'a) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<M3ComponentParam-request>)))
  "Returns string type for a service object of type '<M3ComponentParam-request>"
  "m3_client/M3ComponentParamRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'M3ComponentParam-request)))
  "Returns string type for a service object of type 'M3ComponentParam-request"
  "m3_client/M3ComponentParamRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<M3ComponentParam-request>)))
  "Returns md5sum for a message object of type '<M3ComponentParam-request>"
  "25ba3fa9d5d930574c4d72dc4151cd60")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'M3ComponentParam-request)))
  "Returns md5sum for a message object of type 'M3ComponentParam-request"
  "25ba3fa9d5d930574c4d72dc4151cd60")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<M3ComponentParam-request>)))
  "Returns full string definition for message of type '<M3ComponentParam-request>"
  (cl:format cl:nil "int32 a~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'M3ComponentParam-request)))
  "Returns full string definition for message of type 'M3ComponentParam-request"
  (cl:format cl:nil "int32 a~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <M3ComponentParam-request>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <M3ComponentParam-request>))
  "Converts a ROS message object to a list"
  (cl:list 'M3ComponentParam-request
    (cl:cons ':a (a msg))
))
;//! \htmlinclude M3ComponentParam-response.msg.html

(cl:defclass <M3ComponentParam-response> (roslisp-msg-protocol:ros-message)
  ((b
    :reader b
    :initarg :b
    :type cl:integer
    :initform 0))
)

(cl:defclass M3ComponentParam-response (<M3ComponentParam-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <M3ComponentParam-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'M3ComponentParam-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name m3_client-srv:<M3ComponentParam-response> is deprecated: use m3_client-srv:M3ComponentParam-response instead.")))

(cl:ensure-generic-function 'b-val :lambda-list '(m))
(cl:defmethod b-val ((m <M3ComponentParam-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader m3_client-srv:b-val is deprecated.  Use m3_client-srv:b instead.")
  (b m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <M3ComponentParam-response>) ostream)
  "Serializes a message object of type '<M3ComponentParam-response>"
  (cl:let* ((signed (cl:slot-value msg 'b)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <M3ComponentParam-response>) istream)
  "Deserializes a message object of type '<M3ComponentParam-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'b) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<M3ComponentParam-response>)))
  "Returns string type for a service object of type '<M3ComponentParam-response>"
  "m3_client/M3ComponentParamResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'M3ComponentParam-response)))
  "Returns string type for a service object of type 'M3ComponentParam-response"
  "m3_client/M3ComponentParamResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<M3ComponentParam-response>)))
  "Returns md5sum for a message object of type '<M3ComponentParam-response>"
  "25ba3fa9d5d930574c4d72dc4151cd60")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'M3ComponentParam-response)))
  "Returns md5sum for a message object of type 'M3ComponentParam-response"
  "25ba3fa9d5d930574c4d72dc4151cd60")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<M3ComponentParam-response>)))
  "Returns full string definition for message of type '<M3ComponentParam-response>"
  (cl:format cl:nil "int32 b~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'M3ComponentParam-response)))
  "Returns full string definition for message of type 'M3ComponentParam-response"
  (cl:format cl:nil "int32 b~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <M3ComponentParam-response>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <M3ComponentParam-response>))
  "Converts a ROS message object to a list"
  (cl:list 'M3ComponentParam-response
    (cl:cons ':b (b msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'M3ComponentParam)))
  'M3ComponentParam-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'M3ComponentParam)))
  'M3ComponentParam-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'M3ComponentParam)))
  "Returns string type for a service object of type '<M3ComponentParam>"
  "m3_client/M3ComponentParam")
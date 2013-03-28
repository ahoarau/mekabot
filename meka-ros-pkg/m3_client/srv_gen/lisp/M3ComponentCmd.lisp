; Auto-generated. Do not edit!


(cl:in-package m3_client-srv)


;//! \htmlinclude M3ComponentCmd-request.msg.html

(cl:defclass <M3ComponentCmd-request> (roslisp-msg-protocol:ros-message)
  ((a
    :reader a
    :initarg :a
    :type cl:integer
    :initform 0))
)

(cl:defclass M3ComponentCmd-request (<M3ComponentCmd-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <M3ComponentCmd-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'M3ComponentCmd-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name m3_client-srv:<M3ComponentCmd-request> is deprecated: use m3_client-srv:M3ComponentCmd-request instead.")))

(cl:ensure-generic-function 'a-val :lambda-list '(m))
(cl:defmethod a-val ((m <M3ComponentCmd-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader m3_client-srv:a-val is deprecated.  Use m3_client-srv:a instead.")
  (a m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <M3ComponentCmd-request>) ostream)
  "Serializes a message object of type '<M3ComponentCmd-request>"
  (cl:let* ((signed (cl:slot-value msg 'a)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <M3ComponentCmd-request>) istream)
  "Deserializes a message object of type '<M3ComponentCmd-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'a) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<M3ComponentCmd-request>)))
  "Returns string type for a service object of type '<M3ComponentCmd-request>"
  "m3_client/M3ComponentCmdRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'M3ComponentCmd-request)))
  "Returns string type for a service object of type 'M3ComponentCmd-request"
  "m3_client/M3ComponentCmdRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<M3ComponentCmd-request>)))
  "Returns md5sum for a message object of type '<M3ComponentCmd-request>"
  "25ba3fa9d5d930574c4d72dc4151cd60")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'M3ComponentCmd-request)))
  "Returns md5sum for a message object of type 'M3ComponentCmd-request"
  "25ba3fa9d5d930574c4d72dc4151cd60")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<M3ComponentCmd-request>)))
  "Returns full string definition for message of type '<M3ComponentCmd-request>"
  (cl:format cl:nil "int32 a~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'M3ComponentCmd-request)))
  "Returns full string definition for message of type 'M3ComponentCmd-request"
  (cl:format cl:nil "int32 a~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <M3ComponentCmd-request>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <M3ComponentCmd-request>))
  "Converts a ROS message object to a list"
  (cl:list 'M3ComponentCmd-request
    (cl:cons ':a (a msg))
))
;//! \htmlinclude M3ComponentCmd-response.msg.html

(cl:defclass <M3ComponentCmd-response> (roslisp-msg-protocol:ros-message)
  ((b
    :reader b
    :initarg :b
    :type cl:integer
    :initform 0))
)

(cl:defclass M3ComponentCmd-response (<M3ComponentCmd-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <M3ComponentCmd-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'M3ComponentCmd-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name m3_client-srv:<M3ComponentCmd-response> is deprecated: use m3_client-srv:M3ComponentCmd-response instead.")))

(cl:ensure-generic-function 'b-val :lambda-list '(m))
(cl:defmethod b-val ((m <M3ComponentCmd-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader m3_client-srv:b-val is deprecated.  Use m3_client-srv:b instead.")
  (b m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <M3ComponentCmd-response>) ostream)
  "Serializes a message object of type '<M3ComponentCmd-response>"
  (cl:let* ((signed (cl:slot-value msg 'b)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <M3ComponentCmd-response>) istream)
  "Deserializes a message object of type '<M3ComponentCmd-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'b) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<M3ComponentCmd-response>)))
  "Returns string type for a service object of type '<M3ComponentCmd-response>"
  "m3_client/M3ComponentCmdResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'M3ComponentCmd-response)))
  "Returns string type for a service object of type 'M3ComponentCmd-response"
  "m3_client/M3ComponentCmdResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<M3ComponentCmd-response>)))
  "Returns md5sum for a message object of type '<M3ComponentCmd-response>"
  "25ba3fa9d5d930574c4d72dc4151cd60")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'M3ComponentCmd-response)))
  "Returns md5sum for a message object of type 'M3ComponentCmd-response"
  "25ba3fa9d5d930574c4d72dc4151cd60")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<M3ComponentCmd-response>)))
  "Returns full string definition for message of type '<M3ComponentCmd-response>"
  (cl:format cl:nil "int32 b~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'M3ComponentCmd-response)))
  "Returns full string definition for message of type 'M3ComponentCmd-response"
  (cl:format cl:nil "int32 b~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <M3ComponentCmd-response>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <M3ComponentCmd-response>))
  "Converts a ROS message object to a list"
  (cl:list 'M3ComponentCmd-response
    (cl:cons ':b (b msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'M3ComponentCmd)))
  'M3ComponentCmd-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'M3ComponentCmd)))
  'M3ComponentCmd-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'M3ComponentCmd)))
  "Returns string type for a service object of type '<M3ComponentCmd>"
  "m3_client/M3ComponentCmd")
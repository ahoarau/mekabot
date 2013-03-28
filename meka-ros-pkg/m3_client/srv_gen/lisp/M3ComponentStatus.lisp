; Auto-generated. Do not edit!


(cl:in-package m3_client-srv)


;//! \htmlinclude M3ComponentStatus-request.msg.html

(cl:defclass <M3ComponentStatus-request> (roslisp-msg-protocol:ros-message)
  ((a
    :reader a
    :initarg :a
    :type cl:integer
    :initform 0))
)

(cl:defclass M3ComponentStatus-request (<M3ComponentStatus-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <M3ComponentStatus-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'M3ComponentStatus-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name m3_client-srv:<M3ComponentStatus-request> is deprecated: use m3_client-srv:M3ComponentStatus-request instead.")))

(cl:ensure-generic-function 'a-val :lambda-list '(m))
(cl:defmethod a-val ((m <M3ComponentStatus-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader m3_client-srv:a-val is deprecated.  Use m3_client-srv:a instead.")
  (a m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <M3ComponentStatus-request>) ostream)
  "Serializes a message object of type '<M3ComponentStatus-request>"
  (cl:let* ((signed (cl:slot-value msg 'a)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <M3ComponentStatus-request>) istream)
  "Deserializes a message object of type '<M3ComponentStatus-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'a) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<M3ComponentStatus-request>)))
  "Returns string type for a service object of type '<M3ComponentStatus-request>"
  "m3_client/M3ComponentStatusRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'M3ComponentStatus-request)))
  "Returns string type for a service object of type 'M3ComponentStatus-request"
  "m3_client/M3ComponentStatusRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<M3ComponentStatus-request>)))
  "Returns md5sum for a message object of type '<M3ComponentStatus-request>"
  "241529ee0864eca3736d6be302d71b44")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'M3ComponentStatus-request)))
  "Returns md5sum for a message object of type 'M3ComponentStatus-request"
  "241529ee0864eca3736d6be302d71b44")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<M3ComponentStatus-request>)))
  "Returns full string definition for message of type '<M3ComponentStatus-request>"
  (cl:format cl:nil "int32 a~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'M3ComponentStatus-request)))
  "Returns full string definition for message of type 'M3ComponentStatus-request"
  (cl:format cl:nil "int32 a~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <M3ComponentStatus-request>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <M3ComponentStatus-request>))
  "Converts a ROS message object to a list"
  (cl:list 'M3ComponentStatus-request
    (cl:cons ':a (a msg))
))
;//! \htmlinclude M3ComponentStatus-response.msg.html

(cl:defclass <M3ComponentStatus-response> (roslisp-msg-protocol:ros-message)
  ((base
    :reader base
    :initarg :base
    :type m3_client-msg:M3BaseStatus
    :initform (cl:make-instance 'm3_client-msg:M3BaseStatus)))
)

(cl:defclass M3ComponentStatus-response (<M3ComponentStatus-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <M3ComponentStatus-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'M3ComponentStatus-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name m3_client-srv:<M3ComponentStatus-response> is deprecated: use m3_client-srv:M3ComponentStatus-response instead.")))

(cl:ensure-generic-function 'base-val :lambda-list '(m))
(cl:defmethod base-val ((m <M3ComponentStatus-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader m3_client-srv:base-val is deprecated.  Use m3_client-srv:base instead.")
  (base m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <M3ComponentStatus-response>) ostream)
  "Serializes a message object of type '<M3ComponentStatus-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'base) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <M3ComponentStatus-response>) istream)
  "Deserializes a message object of type '<M3ComponentStatus-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'base) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<M3ComponentStatus-response>)))
  "Returns string type for a service object of type '<M3ComponentStatus-response>"
  "m3_client/M3ComponentStatusResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'M3ComponentStatus-response)))
  "Returns string type for a service object of type 'M3ComponentStatus-response"
  "m3_client/M3ComponentStatusResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<M3ComponentStatus-response>)))
  "Returns md5sum for a message object of type '<M3ComponentStatus-response>"
  "241529ee0864eca3736d6be302d71b44")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'M3ComponentStatus-response)))
  "Returns md5sum for a message object of type 'M3ComponentStatus-response"
  "241529ee0864eca3736d6be302d71b44")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<M3ComponentStatus-response>)))
  "Returns full string definition for message of type '<M3ComponentStatus-response>"
  (cl:format cl:nil "M3BaseStatus base~%~%================================================================================~%MSG: m3_client/M3BaseStatus~%string name~%uint8 state~%int64 timestamp~%string rate~%string version~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'M3ComponentStatus-response)))
  "Returns full string definition for message of type 'M3ComponentStatus-response"
  (cl:format cl:nil "M3BaseStatus base~%~%================================================================================~%MSG: m3_client/M3BaseStatus~%string name~%uint8 state~%int64 timestamp~%string rate~%string version~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <M3ComponentStatus-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'base))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <M3ComponentStatus-response>))
  "Converts a ROS message object to a list"
  (cl:list 'M3ComponentStatus-response
    (cl:cons ':base (base msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'M3ComponentStatus)))
  'M3ComponentStatus-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'M3ComponentStatus)))
  'M3ComponentStatus-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'M3ComponentStatus)))
  "Returns string type for a service object of type '<M3ComponentStatus>"
  "m3_client/M3ComponentStatus")
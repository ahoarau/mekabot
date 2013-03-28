; Auto-generated. Do not edit!


(cl:in-package m3_client-srv)


;//! \htmlinclude M3LoadX6Cmd-request.msg.html

(cl:defclass <M3LoadX6Cmd-request> (roslisp-msg-protocol:ros-message)
  ((request
    :reader request
    :initarg :request
    :type cl:integer
    :initform 0))
)

(cl:defclass M3LoadX6Cmd-request (<M3LoadX6Cmd-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <M3LoadX6Cmd-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'M3LoadX6Cmd-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name m3_client-srv:<M3LoadX6Cmd-request> is deprecated: use m3_client-srv:M3LoadX6Cmd-request instead.")))

(cl:ensure-generic-function 'request-val :lambda-list '(m))
(cl:defmethod request-val ((m <M3LoadX6Cmd-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader m3_client-srv:request-val is deprecated.  Use m3_client-srv:request instead.")
  (request m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <M3LoadX6Cmd-request>) ostream)
  "Serializes a message object of type '<M3LoadX6Cmd-request>"
  (cl:let* ((signed (cl:slot-value msg 'request)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <M3LoadX6Cmd-request>) istream)
  "Deserializes a message object of type '<M3LoadX6Cmd-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'request) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<M3LoadX6Cmd-request>)))
  "Returns string type for a service object of type '<M3LoadX6Cmd-request>"
  "m3_client/M3LoadX6CmdRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'M3LoadX6Cmd-request)))
  "Returns string type for a service object of type 'M3LoadX6Cmd-request"
  "m3_client/M3LoadX6CmdRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<M3LoadX6Cmd-request>)))
  "Returns md5sum for a message object of type '<M3LoadX6Cmd-request>"
  "51edd9dfd50014fde2b589cbf77706aa")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'M3LoadX6Cmd-request)))
  "Returns md5sum for a message object of type 'M3LoadX6Cmd-request"
  "51edd9dfd50014fde2b589cbf77706aa")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<M3LoadX6Cmd-request>)))
  "Returns full string definition for message of type '<M3LoadX6Cmd-request>"
  (cl:format cl:nil "int32 request~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'M3LoadX6Cmd-request)))
  "Returns full string definition for message of type 'M3LoadX6Cmd-request"
  (cl:format cl:nil "int32 request~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <M3LoadX6Cmd-request>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <M3LoadX6Cmd-request>))
  "Converts a ROS message object to a list"
  (cl:list 'M3LoadX6Cmd-request
    (cl:cons ':request (request msg))
))
;//! \htmlinclude M3LoadX6Cmd-response.msg.html

(cl:defclass <M3LoadX6Cmd-response> (roslisp-msg-protocol:ros-message)
  ((response
    :reader response
    :initarg :response
    :type cl:integer
    :initform 0))
)

(cl:defclass M3LoadX6Cmd-response (<M3LoadX6Cmd-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <M3LoadX6Cmd-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'M3LoadX6Cmd-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name m3_client-srv:<M3LoadX6Cmd-response> is deprecated: use m3_client-srv:M3LoadX6Cmd-response instead.")))

(cl:ensure-generic-function 'response-val :lambda-list '(m))
(cl:defmethod response-val ((m <M3LoadX6Cmd-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader m3_client-srv:response-val is deprecated.  Use m3_client-srv:response instead.")
  (response m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <M3LoadX6Cmd-response>) ostream)
  "Serializes a message object of type '<M3LoadX6Cmd-response>"
  (cl:let* ((signed (cl:slot-value msg 'response)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <M3LoadX6Cmd-response>) istream)
  "Deserializes a message object of type '<M3LoadX6Cmd-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'response) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<M3LoadX6Cmd-response>)))
  "Returns string type for a service object of type '<M3LoadX6Cmd-response>"
  "m3_client/M3LoadX6CmdResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'M3LoadX6Cmd-response)))
  "Returns string type for a service object of type 'M3LoadX6Cmd-response"
  "m3_client/M3LoadX6CmdResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<M3LoadX6Cmd-response>)))
  "Returns md5sum for a message object of type '<M3LoadX6Cmd-response>"
  "51edd9dfd50014fde2b589cbf77706aa")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'M3LoadX6Cmd-response)))
  "Returns md5sum for a message object of type 'M3LoadX6Cmd-response"
  "51edd9dfd50014fde2b589cbf77706aa")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<M3LoadX6Cmd-response>)))
  "Returns full string definition for message of type '<M3LoadX6Cmd-response>"
  (cl:format cl:nil "int32 response~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'M3LoadX6Cmd-response)))
  "Returns full string definition for message of type 'M3LoadX6Cmd-response"
  (cl:format cl:nil "int32 response~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <M3LoadX6Cmd-response>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <M3LoadX6Cmd-response>))
  "Converts a ROS message object to a list"
  (cl:list 'M3LoadX6Cmd-response
    (cl:cons ':response (response msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'M3LoadX6Cmd)))
  'M3LoadX6Cmd-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'M3LoadX6Cmd)))
  'M3LoadX6Cmd-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'M3LoadX6Cmd)))
  "Returns string type for a service object of type '<M3LoadX6Cmd>"
  "m3_client/M3LoadX6Cmd")
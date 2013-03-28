; Auto-generated. Do not edit!


(cl:in-package m3_client-srv)


;//! \htmlinclude M3JointArrayParam-request.msg.html

(cl:defclass <M3JointArrayParam-request> (roslisp-msg-protocol:ros-message)
  ((request
    :reader request
    :initarg :request
    :type cl:integer
    :initform 0))
)

(cl:defclass M3JointArrayParam-request (<M3JointArrayParam-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <M3JointArrayParam-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'M3JointArrayParam-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name m3_client-srv:<M3JointArrayParam-request> is deprecated: use m3_client-srv:M3JointArrayParam-request instead.")))

(cl:ensure-generic-function 'request-val :lambda-list '(m))
(cl:defmethod request-val ((m <M3JointArrayParam-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader m3_client-srv:request-val is deprecated.  Use m3_client-srv:request instead.")
  (request m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <M3JointArrayParam-request>) ostream)
  "Serializes a message object of type '<M3JointArrayParam-request>"
  (cl:let* ((signed (cl:slot-value msg 'request)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <M3JointArrayParam-request>) istream)
  "Deserializes a message object of type '<M3JointArrayParam-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'request) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<M3JointArrayParam-request>)))
  "Returns string type for a service object of type '<M3JointArrayParam-request>"
  "m3_client/M3JointArrayParamRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'M3JointArrayParam-request)))
  "Returns string type for a service object of type 'M3JointArrayParam-request"
  "m3_client/M3JointArrayParamRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<M3JointArrayParam-request>)))
  "Returns md5sum for a message object of type '<M3JointArrayParam-request>"
  "51edd9dfd50014fde2b589cbf77706aa")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'M3JointArrayParam-request)))
  "Returns md5sum for a message object of type 'M3JointArrayParam-request"
  "51edd9dfd50014fde2b589cbf77706aa")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<M3JointArrayParam-request>)))
  "Returns full string definition for message of type '<M3JointArrayParam-request>"
  (cl:format cl:nil "int32 request~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'M3JointArrayParam-request)))
  "Returns full string definition for message of type 'M3JointArrayParam-request"
  (cl:format cl:nil "int32 request~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <M3JointArrayParam-request>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <M3JointArrayParam-request>))
  "Converts a ROS message object to a list"
  (cl:list 'M3JointArrayParam-request
    (cl:cons ':request (request msg))
))
;//! \htmlinclude M3JointArrayParam-response.msg.html

(cl:defclass <M3JointArrayParam-response> (roslisp-msg-protocol:ros-message)
  ((response
    :reader response
    :initarg :response
    :type cl:integer
    :initform 0))
)

(cl:defclass M3JointArrayParam-response (<M3JointArrayParam-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <M3JointArrayParam-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'M3JointArrayParam-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name m3_client-srv:<M3JointArrayParam-response> is deprecated: use m3_client-srv:M3JointArrayParam-response instead.")))

(cl:ensure-generic-function 'response-val :lambda-list '(m))
(cl:defmethod response-val ((m <M3JointArrayParam-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader m3_client-srv:response-val is deprecated.  Use m3_client-srv:response instead.")
  (response m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <M3JointArrayParam-response>) ostream)
  "Serializes a message object of type '<M3JointArrayParam-response>"
  (cl:let* ((signed (cl:slot-value msg 'response)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <M3JointArrayParam-response>) istream)
  "Deserializes a message object of type '<M3JointArrayParam-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'response) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<M3JointArrayParam-response>)))
  "Returns string type for a service object of type '<M3JointArrayParam-response>"
  "m3_client/M3JointArrayParamResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'M3JointArrayParam-response)))
  "Returns string type for a service object of type 'M3JointArrayParam-response"
  "m3_client/M3JointArrayParamResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<M3JointArrayParam-response>)))
  "Returns md5sum for a message object of type '<M3JointArrayParam-response>"
  "51edd9dfd50014fde2b589cbf77706aa")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'M3JointArrayParam-response)))
  "Returns md5sum for a message object of type 'M3JointArrayParam-response"
  "51edd9dfd50014fde2b589cbf77706aa")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<M3JointArrayParam-response>)))
  "Returns full string definition for message of type '<M3JointArrayParam-response>"
  (cl:format cl:nil "int32 response~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'M3JointArrayParam-response)))
  "Returns full string definition for message of type 'M3JointArrayParam-response"
  (cl:format cl:nil "int32 response~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <M3JointArrayParam-response>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <M3JointArrayParam-response>))
  "Converts a ROS message object to a list"
  (cl:list 'M3JointArrayParam-response
    (cl:cons ':response (response msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'M3JointArrayParam)))
  'M3JointArrayParam-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'M3JointArrayParam)))
  'M3JointArrayParam-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'M3JointArrayParam)))
  "Returns string type for a service object of type '<M3JointArrayParam>"
  "m3_client/M3JointArrayParam")
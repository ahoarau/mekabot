; Auto-generated. Do not edit!


(cl:in-package m3_client-srv)


;//! \htmlinclude M3HumanoidParam-request.msg.html

(cl:defclass <M3HumanoidParam-request> (roslisp-msg-protocol:ros-message)
  ((chain
    :reader chain
    :initarg :chain
    :type cl:fixnum
    :initform 0)
   (payload_mass
    :reader payload_mass
    :initarg :payload_mass
    :type cl:float
    :initform 0.0)
   (payload_com
    :reader payload_com
    :initarg :payload_com
    :type (cl:vector cl:float)
   :initform (cl:make-array 3 :element-type 'cl:float :initial-element 0.0))
   (payload_inertia
    :reader payload_inertia
    :initarg :payload_inertia
    :type (cl:vector cl:float)
   :initform (cl:make-array 6 :element-type 'cl:float :initial-element 0.0))
   (use_velocities
    :reader use_velocities
    :initarg :use_velocities
    :type cl:boolean
    :initform cl:nil)
   (use_accelerations
    :reader use_accelerations
    :initarg :use_accelerations
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass M3HumanoidParam-request (<M3HumanoidParam-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <M3HumanoidParam-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'M3HumanoidParam-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name m3_client-srv:<M3HumanoidParam-request> is deprecated: use m3_client-srv:M3HumanoidParam-request instead.")))

(cl:ensure-generic-function 'chain-val :lambda-list '(m))
(cl:defmethod chain-val ((m <M3HumanoidParam-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader m3_client-srv:chain-val is deprecated.  Use m3_client-srv:chain instead.")
  (chain m))

(cl:ensure-generic-function 'payload_mass-val :lambda-list '(m))
(cl:defmethod payload_mass-val ((m <M3HumanoidParam-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader m3_client-srv:payload_mass-val is deprecated.  Use m3_client-srv:payload_mass instead.")
  (payload_mass m))

(cl:ensure-generic-function 'payload_com-val :lambda-list '(m))
(cl:defmethod payload_com-val ((m <M3HumanoidParam-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader m3_client-srv:payload_com-val is deprecated.  Use m3_client-srv:payload_com instead.")
  (payload_com m))

(cl:ensure-generic-function 'payload_inertia-val :lambda-list '(m))
(cl:defmethod payload_inertia-val ((m <M3HumanoidParam-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader m3_client-srv:payload_inertia-val is deprecated.  Use m3_client-srv:payload_inertia instead.")
  (payload_inertia m))

(cl:ensure-generic-function 'use_velocities-val :lambda-list '(m))
(cl:defmethod use_velocities-val ((m <M3HumanoidParam-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader m3_client-srv:use_velocities-val is deprecated.  Use m3_client-srv:use_velocities instead.")
  (use_velocities m))

(cl:ensure-generic-function 'use_accelerations-val :lambda-list '(m))
(cl:defmethod use_accelerations-val ((m <M3HumanoidParam-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader m3_client-srv:use_accelerations-val is deprecated.  Use m3_client-srv:use_accelerations instead.")
  (use_accelerations m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <M3HumanoidParam-request>) ostream)
  "Serializes a message object of type '<M3HumanoidParam-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'chain)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'payload_mass))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'payload_com))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'payload_inertia))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'use_velocities) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'use_accelerations) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <M3HumanoidParam-request>) istream)
  "Deserializes a message object of type '<M3HumanoidParam-request>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'chain)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'payload_mass) (roslisp-utils:decode-single-float-bits bits)))
  (cl:setf (cl:slot-value msg 'payload_com) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'payload_com)))
    (cl:dotimes (i 3)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  (cl:setf (cl:slot-value msg 'payload_inertia) (cl:make-array 6))
  (cl:let ((vals (cl:slot-value msg 'payload_inertia)))
    (cl:dotimes (i 6)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
    (cl:setf (cl:slot-value msg 'use_velocities) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'use_accelerations) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<M3HumanoidParam-request>)))
  "Returns string type for a service object of type '<M3HumanoidParam-request>"
  "m3_client/M3HumanoidParamRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'M3HumanoidParam-request)))
  "Returns string type for a service object of type 'M3HumanoidParam-request"
  "m3_client/M3HumanoidParamRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<M3HumanoidParam-request>)))
  "Returns md5sum for a message object of type '<M3HumanoidParam-request>"
  "34cb417585df77f14f1029d4fc16441a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'M3HumanoidParam-request)))
  "Returns md5sum for a message object of type 'M3HumanoidParam-request"
  "34cb417585df77f14f1029d4fc16441a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<M3HumanoidParam-request>)))
  "Returns full string definition for message of type '<M3HumanoidParam-request>"
  (cl:format cl:nil "uint8 chain~%float32 payload_mass~%float32[3] payload_com~%float32[6] payload_inertia~%bool use_velocities~%bool use_accelerations~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'M3HumanoidParam-request)))
  "Returns full string definition for message of type 'M3HumanoidParam-request"
  (cl:format cl:nil "uint8 chain~%float32 payload_mass~%float32[3] payload_com~%float32[6] payload_inertia~%bool use_velocities~%bool use_accelerations~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <M3HumanoidParam-request>))
  (cl:+ 0
     1
     4
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'payload_com) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'payload_inertia) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <M3HumanoidParam-request>))
  "Converts a ROS message object to a list"
  (cl:list 'M3HumanoidParam-request
    (cl:cons ':chain (chain msg))
    (cl:cons ':payload_mass (payload_mass msg))
    (cl:cons ':payload_com (payload_com msg))
    (cl:cons ':payload_inertia (payload_inertia msg))
    (cl:cons ':use_velocities (use_velocities msg))
    (cl:cons ':use_accelerations (use_accelerations msg))
))
;//! \htmlinclude M3HumanoidParam-response.msg.html

(cl:defclass <M3HumanoidParam-response> (roslisp-msg-protocol:ros-message)
  ((response
    :reader response
    :initarg :response
    :type cl:integer
    :initform 0))
)

(cl:defclass M3HumanoidParam-response (<M3HumanoidParam-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <M3HumanoidParam-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'M3HumanoidParam-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name m3_client-srv:<M3HumanoidParam-response> is deprecated: use m3_client-srv:M3HumanoidParam-response instead.")))

(cl:ensure-generic-function 'response-val :lambda-list '(m))
(cl:defmethod response-val ((m <M3HumanoidParam-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader m3_client-srv:response-val is deprecated.  Use m3_client-srv:response instead.")
  (response m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <M3HumanoidParam-response>) ostream)
  "Serializes a message object of type '<M3HumanoidParam-response>"
  (cl:let* ((signed (cl:slot-value msg 'response)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <M3HumanoidParam-response>) istream)
  "Deserializes a message object of type '<M3HumanoidParam-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'response) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<M3HumanoidParam-response>)))
  "Returns string type for a service object of type '<M3HumanoidParam-response>"
  "m3_client/M3HumanoidParamResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'M3HumanoidParam-response)))
  "Returns string type for a service object of type 'M3HumanoidParam-response"
  "m3_client/M3HumanoidParamResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<M3HumanoidParam-response>)))
  "Returns md5sum for a message object of type '<M3HumanoidParam-response>"
  "34cb417585df77f14f1029d4fc16441a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'M3HumanoidParam-response)))
  "Returns md5sum for a message object of type 'M3HumanoidParam-response"
  "34cb417585df77f14f1029d4fc16441a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<M3HumanoidParam-response>)))
  "Returns full string definition for message of type '<M3HumanoidParam-response>"
  (cl:format cl:nil "int32 response~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'M3HumanoidParam-response)))
  "Returns full string definition for message of type 'M3HumanoidParam-response"
  (cl:format cl:nil "int32 response~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <M3HumanoidParam-response>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <M3HumanoidParam-response>))
  "Converts a ROS message object to a list"
  (cl:list 'M3HumanoidParam-response
    (cl:cons ':response (response msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'M3HumanoidParam)))
  'M3HumanoidParam-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'M3HumanoidParam)))
  'M3HumanoidParam-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'M3HumanoidParam)))
  "Returns string type for a service object of type '<M3HumanoidParam>"
  "m3_client/M3HumanoidParam")
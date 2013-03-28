; Auto-generated. Do not edit!


(cl:in-package m3_client-msg)


;//! \htmlinclude M3OmnibaseJoy.msg.html

(cl:defclass <M3OmnibaseJoy> (roslisp-msg-protocol:ros-message)
  ((x
    :reader x
    :initarg :x
    :type cl:float
    :initform 0.0)
   (y
    :reader y
    :initarg :y
    :type cl:float
    :initform 0.0)
   (yaw
    :reader yaw
    :initarg :yaw
    :type cl:float
    :initform 0.0)
   (button
    :reader button
    :initarg :button
    :type cl:float
    :initform 0.0)
   (z
    :reader z
    :initarg :z
    :type cl:float
    :initform 0.0))
)

(cl:defclass M3OmnibaseJoy (<M3OmnibaseJoy>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <M3OmnibaseJoy>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'M3OmnibaseJoy)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name m3_client-msg:<M3OmnibaseJoy> is deprecated: use m3_client-msg:M3OmnibaseJoy instead.")))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <M3OmnibaseJoy>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader m3_client-msg:x-val is deprecated.  Use m3_client-msg:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <M3OmnibaseJoy>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader m3_client-msg:y-val is deprecated.  Use m3_client-msg:y instead.")
  (y m))

(cl:ensure-generic-function 'yaw-val :lambda-list '(m))
(cl:defmethod yaw-val ((m <M3OmnibaseJoy>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader m3_client-msg:yaw-val is deprecated.  Use m3_client-msg:yaw instead.")
  (yaw m))

(cl:ensure-generic-function 'button-val :lambda-list '(m))
(cl:defmethod button-val ((m <M3OmnibaseJoy>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader m3_client-msg:button-val is deprecated.  Use m3_client-msg:button instead.")
  (button m))

(cl:ensure-generic-function 'z-val :lambda-list '(m))
(cl:defmethod z-val ((m <M3OmnibaseJoy>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader m3_client-msg:z-val is deprecated.  Use m3_client-msg:z instead.")
  (z m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <M3OmnibaseJoy>) ostream)
  "Serializes a message object of type '<M3OmnibaseJoy>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'yaw))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'button))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'z))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <M3OmnibaseJoy>) istream)
  "Deserializes a message object of type '<M3OmnibaseJoy>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'yaw) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'button) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'z) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<M3OmnibaseJoy>)))
  "Returns string type for a message object of type '<M3OmnibaseJoy>"
  "m3_client/M3OmnibaseJoy")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'M3OmnibaseJoy)))
  "Returns string type for a message object of type 'M3OmnibaseJoy"
  "m3_client/M3OmnibaseJoy")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<M3OmnibaseJoy>)))
  "Returns md5sum for a message object of type '<M3OmnibaseJoy>"
  "6719502035b93742f7b2585c261584a9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'M3OmnibaseJoy)))
  "Returns md5sum for a message object of type 'M3OmnibaseJoy"
  "6719502035b93742f7b2585c261584a9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<M3OmnibaseJoy>)))
  "Returns full string definition for message of type '<M3OmnibaseJoy>"
  (cl:format cl:nil "float32 x~%float32 y~%float32 yaw~%float32 button~%float32 z~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'M3OmnibaseJoy)))
  "Returns full string definition for message of type 'M3OmnibaseJoy"
  (cl:format cl:nil "float32 x~%float32 y~%float32 yaw~%float32 button~%float32 z~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <M3OmnibaseJoy>))
  (cl:+ 0
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <M3OmnibaseJoy>))
  "Converts a ROS message object to a list"
  (cl:list 'M3OmnibaseJoy
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
    (cl:cons ':yaw (yaw msg))
    (cl:cons ':button (button msg))
    (cl:cons ':z (z msg))
))

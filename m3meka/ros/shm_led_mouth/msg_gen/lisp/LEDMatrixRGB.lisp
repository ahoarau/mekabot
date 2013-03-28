; Auto-generated. Do not edit!


(cl:in-package shm_led_mouth-msg)


;//! \htmlinclude LEDMatrixRGB.msg.html

(cl:defclass <LEDMatrixRGB> (roslisp-msg-protocol:ros-message)
  ((r
    :reader r
    :initarg :r
    :type cl:integer
    :initform 0)
   (g
    :reader g
    :initarg :g
    :type cl:integer
    :initform 0)
   (b
    :reader b
    :initarg :b
    :type cl:integer
    :initform 0))
)

(cl:defclass LEDMatrixRGB (<LEDMatrixRGB>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <LEDMatrixRGB>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'LEDMatrixRGB)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name shm_led_mouth-msg:<LEDMatrixRGB> is deprecated: use shm_led_mouth-msg:LEDMatrixRGB instead.")))

(cl:ensure-generic-function 'r-val :lambda-list '(m))
(cl:defmethod r-val ((m <LEDMatrixRGB>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader shm_led_mouth-msg:r-val is deprecated.  Use shm_led_mouth-msg:r instead.")
  (r m))

(cl:ensure-generic-function 'g-val :lambda-list '(m))
(cl:defmethod g-val ((m <LEDMatrixRGB>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader shm_led_mouth-msg:g-val is deprecated.  Use shm_led_mouth-msg:g instead.")
  (g m))

(cl:ensure-generic-function 'b-val :lambda-list '(m))
(cl:defmethod b-val ((m <LEDMatrixRGB>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader shm_led_mouth-msg:b-val is deprecated.  Use shm_led_mouth-msg:b instead.")
  (b m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <LEDMatrixRGB>) ostream)
  "Serializes a message object of type '<LEDMatrixRGB>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'r)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'r)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'r)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'r)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'g)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'g)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'g)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'g)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'b)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'b)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'b)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'b)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <LEDMatrixRGB>) istream)
  "Deserializes a message object of type '<LEDMatrixRGB>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'r)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'r)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'r)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'r)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'g)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'g)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'g)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'g)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'b)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'b)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'b)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'b)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<LEDMatrixRGB>)))
  "Returns string type for a message object of type '<LEDMatrixRGB>"
  "shm_led_mouth/LEDMatrixRGB")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LEDMatrixRGB)))
  "Returns string type for a message object of type 'LEDMatrixRGB"
  "shm_led_mouth/LEDMatrixRGB")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<LEDMatrixRGB>)))
  "Returns md5sum for a message object of type '<LEDMatrixRGB>"
  "1d502d8fbb4c6ae56a393ae619466e53")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'LEDMatrixRGB)))
  "Returns md5sum for a message object of type 'LEDMatrixRGB"
  "1d502d8fbb4c6ae56a393ae619466e53")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<LEDMatrixRGB>)))
  "Returns full string definition for message of type '<LEDMatrixRGB>"
  (cl:format cl:nil "uint32 r~%uint32 g~%uint32 b~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'LEDMatrixRGB)))
  "Returns full string definition for message of type 'LEDMatrixRGB"
  (cl:format cl:nil "uint32 r~%uint32 g~%uint32 b~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <LEDMatrixRGB>))
  (cl:+ 0
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <LEDMatrixRGB>))
  "Converts a ROS message object to a list"
  (cl:list 'LEDMatrixRGB
    (cl:cons ':r (r msg))
    (cl:cons ':g (g msg))
    (cl:cons ':b (b msg))
))

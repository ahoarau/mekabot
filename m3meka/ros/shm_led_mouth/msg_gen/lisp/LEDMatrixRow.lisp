; Auto-generated. Do not edit!


(cl:in-package shm_led_mouth-msg)


;//! \htmlinclude LEDMatrixRow.msg.html

(cl:defclass <LEDMatrixRow> (roslisp-msg-protocol:ros-message)
  ((column
    :reader column
    :initarg :column
    :type (cl:vector shm_led_mouth-msg:LEDMatrixRGB)
   :initform (cl:make-array 0 :element-type 'shm_led_mouth-msg:LEDMatrixRGB :initial-element (cl:make-instance 'shm_led_mouth-msg:LEDMatrixRGB))))
)

(cl:defclass LEDMatrixRow (<LEDMatrixRow>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <LEDMatrixRow>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'LEDMatrixRow)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name shm_led_mouth-msg:<LEDMatrixRow> is deprecated: use shm_led_mouth-msg:LEDMatrixRow instead.")))

(cl:ensure-generic-function 'column-val :lambda-list '(m))
(cl:defmethod column-val ((m <LEDMatrixRow>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader shm_led_mouth-msg:column-val is deprecated.  Use shm_led_mouth-msg:column instead.")
  (column m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <LEDMatrixRow>) ostream)
  "Serializes a message object of type '<LEDMatrixRow>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'column))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'column))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <LEDMatrixRow>) istream)
  "Deserializes a message object of type '<LEDMatrixRow>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'column) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'column)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'shm_led_mouth-msg:LEDMatrixRGB))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<LEDMatrixRow>)))
  "Returns string type for a message object of type '<LEDMatrixRow>"
  "shm_led_mouth/LEDMatrixRow")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LEDMatrixRow)))
  "Returns string type for a message object of type 'LEDMatrixRow"
  "shm_led_mouth/LEDMatrixRow")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<LEDMatrixRow>)))
  "Returns md5sum for a message object of type '<LEDMatrixRow>"
  "5a0313057dae76e530e4ef99ceb0eca4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'LEDMatrixRow)))
  "Returns md5sum for a message object of type 'LEDMatrixRow"
  "5a0313057dae76e530e4ef99ceb0eca4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<LEDMatrixRow>)))
  "Returns full string definition for message of type '<LEDMatrixRow>"
  (cl:format cl:nil "LEDMatrixRGB[] column~%================================================================================~%MSG: shm_led_mouth/LEDMatrixRGB~%uint32 r~%uint32 g~%uint32 b~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'LEDMatrixRow)))
  "Returns full string definition for message of type 'LEDMatrixRow"
  (cl:format cl:nil "LEDMatrixRGB[] column~%================================================================================~%MSG: shm_led_mouth/LEDMatrixRGB~%uint32 r~%uint32 g~%uint32 b~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <LEDMatrixRow>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'column) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <LEDMatrixRow>))
  "Converts a ROS message object to a list"
  (cl:list 'LEDMatrixRow
    (cl:cons ':column (column msg))
))

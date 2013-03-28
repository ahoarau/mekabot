; Auto-generated. Do not edit!


(cl:in-package shm_led_mouth-msg)


;//! \htmlinclude LEDMatrixCmd.msg.html

(cl:defclass <LEDMatrixCmd> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (enable
    :reader enable
    :initarg :enable
    :type cl:fixnum
    :initform 0)
   (row
    :reader row
    :initarg :row
    :type (cl:vector shm_led_mouth-msg:LEDMatrixRow)
   :initform (cl:make-array 0 :element-type 'shm_led_mouth-msg:LEDMatrixRow :initial-element (cl:make-instance 'shm_led_mouth-msg:LEDMatrixRow))))
)

(cl:defclass LEDMatrixCmd (<LEDMatrixCmd>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <LEDMatrixCmd>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'LEDMatrixCmd)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name shm_led_mouth-msg:<LEDMatrixCmd> is deprecated: use shm_led_mouth-msg:LEDMatrixCmd instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <LEDMatrixCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader shm_led_mouth-msg:header-val is deprecated.  Use shm_led_mouth-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'enable-val :lambda-list '(m))
(cl:defmethod enable-val ((m <LEDMatrixCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader shm_led_mouth-msg:enable-val is deprecated.  Use shm_led_mouth-msg:enable instead.")
  (enable m))

(cl:ensure-generic-function 'row-val :lambda-list '(m))
(cl:defmethod row-val ((m <LEDMatrixCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader shm_led_mouth-msg:row-val is deprecated.  Use shm_led_mouth-msg:row instead.")
  (row m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <LEDMatrixCmd>) ostream)
  "Serializes a message object of type '<LEDMatrixCmd>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'enable)) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'row))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'row))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <LEDMatrixCmd>) istream)
  "Deserializes a message object of type '<LEDMatrixCmd>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'enable)) (cl:read-byte istream))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'row) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'row)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'shm_led_mouth-msg:LEDMatrixRow))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<LEDMatrixCmd>)))
  "Returns string type for a message object of type '<LEDMatrixCmd>"
  "shm_led_mouth/LEDMatrixCmd")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LEDMatrixCmd)))
  "Returns string type for a message object of type 'LEDMatrixCmd"
  "shm_led_mouth/LEDMatrixCmd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<LEDMatrixCmd>)))
  "Returns md5sum for a message object of type '<LEDMatrixCmd>"
  "e6c5973b6a371aea4cb4dbb5179512f6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'LEDMatrixCmd)))
  "Returns md5sum for a message object of type 'LEDMatrixCmd"
  "e6c5973b6a371aea4cb4dbb5179512f6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<LEDMatrixCmd>)))
  "Returns full string definition for message of type '<LEDMatrixCmd>"
  (cl:format cl:nil "Header header~%uint8 enable~%LEDMatrixRow[] row~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: shm_led_mouth/LEDMatrixRow~%LEDMatrixRGB[] column~%================================================================================~%MSG: shm_led_mouth/LEDMatrixRGB~%uint32 r~%uint32 g~%uint32 b~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'LEDMatrixCmd)))
  "Returns full string definition for message of type 'LEDMatrixCmd"
  (cl:format cl:nil "Header header~%uint8 enable~%LEDMatrixRow[] row~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: shm_led_mouth/LEDMatrixRow~%LEDMatrixRGB[] column~%================================================================================~%MSG: shm_led_mouth/LEDMatrixRGB~%uint32 r~%uint32 g~%uint32 b~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <LEDMatrixCmd>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'row) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <LEDMatrixCmd>))
  "Converts a ROS message object to a list"
  (cl:list 'LEDMatrixCmd
    (cl:cons ':header (header msg))
    (cl:cons ':enable (enable msg))
    (cl:cons ':row (row msg))
))

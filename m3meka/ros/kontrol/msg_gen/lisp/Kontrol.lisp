; Auto-generated. Do not edit!


(cl:in-package kontrol-msg)


;//! \htmlinclude Kontrol.msg.html

(cl:defclass <Kontrol> (roslisp-msg-protocol:ros-message)
  ((sliders
    :reader sliders
    :initarg :sliders
    :type (cl:vector cl:integer)
   :initform (cl:make-array 0 :element-type 'cl:integer :initial-element 0))
   (knobs
    :reader knobs
    :initarg :knobs
    :type (cl:vector cl:integer)
   :initform (cl:make-array 0 :element-type 'cl:integer :initial-element 0))
   (buttons
    :reader buttons
    :initarg :buttons
    :type (cl:vector cl:integer)
   :initform (cl:make-array 0 :element-type 'cl:integer :initial-element 0)))
)

(cl:defclass Kontrol (<Kontrol>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Kontrol>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Kontrol)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kontrol-msg:<Kontrol> is deprecated: use kontrol-msg:Kontrol instead.")))

(cl:ensure-generic-function 'sliders-val :lambda-list '(m))
(cl:defmethod sliders-val ((m <Kontrol>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kontrol-msg:sliders-val is deprecated.  Use kontrol-msg:sliders instead.")
  (sliders m))

(cl:ensure-generic-function 'knobs-val :lambda-list '(m))
(cl:defmethod knobs-val ((m <Kontrol>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kontrol-msg:knobs-val is deprecated.  Use kontrol-msg:knobs instead.")
  (knobs m))

(cl:ensure-generic-function 'buttons-val :lambda-list '(m))
(cl:defmethod buttons-val ((m <Kontrol>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kontrol-msg:buttons-val is deprecated.  Use kontrol-msg:buttons instead.")
  (buttons m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Kontrol>) ostream)
  "Serializes a message object of type '<Kontrol>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'sliders))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    ))
   (cl:slot-value msg 'sliders))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'knobs))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    ))
   (cl:slot-value msg 'knobs))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'buttons))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    ))
   (cl:slot-value msg 'buttons))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Kontrol>) istream)
  "Deserializes a message object of type '<Kontrol>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'sliders) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'sliders)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296)))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'knobs) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'knobs)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296)))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'buttons) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'buttons)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296)))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Kontrol>)))
  "Returns string type for a message object of type '<Kontrol>"
  "kontrol/Kontrol")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Kontrol)))
  "Returns string type for a message object of type 'Kontrol"
  "kontrol/Kontrol")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Kontrol>)))
  "Returns md5sum for a message object of type '<Kontrol>"
  "c1e4678c7964f4fee41470b640a6873e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Kontrol)))
  "Returns md5sum for a message object of type 'Kontrol"
  "c1e4678c7964f4fee41470b640a6873e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Kontrol>)))
  "Returns full string definition for message of type '<Kontrol>"
  (cl:format cl:nil "int32[] sliders~%int32[] knobs~%int32[] buttons~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Kontrol)))
  "Returns full string definition for message of type 'Kontrol"
  (cl:format cl:nil "int32[] sliders~%int32[] knobs~%int32[] buttons~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Kontrol>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'sliders) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'knobs) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'buttons) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Kontrol>))
  "Converts a ROS message object to a list"
  (cl:list 'Kontrol
    (cl:cons ':sliders (sliders msg))
    (cl:cons ':knobs (knobs msg))
    (cl:cons ':buttons (buttons msg))
))

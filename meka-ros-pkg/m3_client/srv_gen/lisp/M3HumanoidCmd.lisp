; Auto-generated. Do not edit!


(cl:in-package m3_client-srv)


;//! \htmlinclude M3HumanoidCmd-request.msg.html

(cl:defclass <M3HumanoidCmd-request> (roslisp-msg-protocol:ros-message)
  ((chain
    :reader chain
    :initarg :chain
    :type cl:fixnum
    :initform 0)
   (tq_desired
    :reader tq_desired
    :initarg :tq_desired
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (ctrl_mode
    :reader ctrl_mode
    :initarg :ctrl_mode
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0))
   (q_stiffness
    :reader q_stiffness
    :initarg :q_stiffness
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (q_desired
    :reader q_desired
    :initarg :q_desired
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (qdot_desired
    :reader qdot_desired
    :initarg :qdot_desired
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (q_slew_rate
    :reader q_slew_rate
    :initarg :q_slew_rate
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (pwm_desired
    :reader pwm_desired
    :initarg :pwm_desired
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (enable_motor
    :reader enable_motor
    :initarg :enable_motor
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass M3HumanoidCmd-request (<M3HumanoidCmd-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <M3HumanoidCmd-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'M3HumanoidCmd-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name m3_client-srv:<M3HumanoidCmd-request> is deprecated: use m3_client-srv:M3HumanoidCmd-request instead.")))

(cl:ensure-generic-function 'chain-val :lambda-list '(m))
(cl:defmethod chain-val ((m <M3HumanoidCmd-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader m3_client-srv:chain-val is deprecated.  Use m3_client-srv:chain instead.")
  (chain m))

(cl:ensure-generic-function 'tq_desired-val :lambda-list '(m))
(cl:defmethod tq_desired-val ((m <M3HumanoidCmd-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader m3_client-srv:tq_desired-val is deprecated.  Use m3_client-srv:tq_desired instead.")
  (tq_desired m))

(cl:ensure-generic-function 'ctrl_mode-val :lambda-list '(m))
(cl:defmethod ctrl_mode-val ((m <M3HumanoidCmd-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader m3_client-srv:ctrl_mode-val is deprecated.  Use m3_client-srv:ctrl_mode instead.")
  (ctrl_mode m))

(cl:ensure-generic-function 'q_stiffness-val :lambda-list '(m))
(cl:defmethod q_stiffness-val ((m <M3HumanoidCmd-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader m3_client-srv:q_stiffness-val is deprecated.  Use m3_client-srv:q_stiffness instead.")
  (q_stiffness m))

(cl:ensure-generic-function 'q_desired-val :lambda-list '(m))
(cl:defmethod q_desired-val ((m <M3HumanoidCmd-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader m3_client-srv:q_desired-val is deprecated.  Use m3_client-srv:q_desired instead.")
  (q_desired m))

(cl:ensure-generic-function 'qdot_desired-val :lambda-list '(m))
(cl:defmethod qdot_desired-val ((m <M3HumanoidCmd-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader m3_client-srv:qdot_desired-val is deprecated.  Use m3_client-srv:qdot_desired instead.")
  (qdot_desired m))

(cl:ensure-generic-function 'q_slew_rate-val :lambda-list '(m))
(cl:defmethod q_slew_rate-val ((m <M3HumanoidCmd-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader m3_client-srv:q_slew_rate-val is deprecated.  Use m3_client-srv:q_slew_rate instead.")
  (q_slew_rate m))

(cl:ensure-generic-function 'pwm_desired-val :lambda-list '(m))
(cl:defmethod pwm_desired-val ((m <M3HumanoidCmd-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader m3_client-srv:pwm_desired-val is deprecated.  Use m3_client-srv:pwm_desired instead.")
  (pwm_desired m))

(cl:ensure-generic-function 'enable_motor-val :lambda-list '(m))
(cl:defmethod enable_motor-val ((m <M3HumanoidCmd-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader m3_client-srv:enable_motor-val is deprecated.  Use m3_client-srv:enable_motor instead.")
  (enable_motor m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <M3HumanoidCmd-request>) ostream)
  "Serializes a message object of type '<M3HumanoidCmd-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'chain)) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'tq_desired))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'tq_desired))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'ctrl_mode))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream))
   (cl:slot-value msg 'ctrl_mode))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'q_stiffness))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'q_stiffness))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'q_desired))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'q_desired))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'qdot_desired))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'qdot_desired))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'q_slew_rate))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'q_slew_rate))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'pwm_desired))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'pwm_desired))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'enable_motor) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <M3HumanoidCmd-request>) istream)
  "Deserializes a message object of type '<M3HumanoidCmd-request>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'chain)) (cl:read-byte istream))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'tq_desired) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'tq_desired)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'ctrl_mode) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'ctrl_mode)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream)))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'q_stiffness) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'q_stiffness)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'q_desired) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'q_desired)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'qdot_desired) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'qdot_desired)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'q_slew_rate) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'q_slew_rate)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'pwm_desired) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'pwm_desired)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
    (cl:setf (cl:slot-value msg 'enable_motor) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<M3HumanoidCmd-request>)))
  "Returns string type for a service object of type '<M3HumanoidCmd-request>"
  "m3_client/M3HumanoidCmdRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'M3HumanoidCmd-request)))
  "Returns string type for a service object of type 'M3HumanoidCmd-request"
  "m3_client/M3HumanoidCmdRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<M3HumanoidCmd-request>)))
  "Returns md5sum for a message object of type '<M3HumanoidCmd-request>"
  "36825d9d1e10e133bf2112ce1e5afc9c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'M3HumanoidCmd-request)))
  "Returns md5sum for a message object of type 'M3HumanoidCmd-request"
  "36825d9d1e10e133bf2112ce1e5afc9c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<M3HumanoidCmd-request>)))
  "Returns full string definition for message of type '<M3HumanoidCmd-request>"
  (cl:format cl:nil "uint8 chain~%float32[] tq_desired~%uint8[] ctrl_mode~%float32[] q_stiffness~%float32[] q_desired~%float32[] qdot_desired~%float32[] q_slew_rate~%float32[] pwm_desired~%bool enable_motor~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'M3HumanoidCmd-request)))
  "Returns full string definition for message of type 'M3HumanoidCmd-request"
  (cl:format cl:nil "uint8 chain~%float32[] tq_desired~%uint8[] ctrl_mode~%float32[] q_stiffness~%float32[] q_desired~%float32[] qdot_desired~%float32[] q_slew_rate~%float32[] pwm_desired~%bool enable_motor~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <M3HumanoidCmd-request>))
  (cl:+ 0
     1
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'tq_desired) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'ctrl_mode) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'q_stiffness) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'q_desired) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'qdot_desired) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'q_slew_rate) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'pwm_desired) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <M3HumanoidCmd-request>))
  "Converts a ROS message object to a list"
  (cl:list 'M3HumanoidCmd-request
    (cl:cons ':chain (chain msg))
    (cl:cons ':tq_desired (tq_desired msg))
    (cl:cons ':ctrl_mode (ctrl_mode msg))
    (cl:cons ':q_stiffness (q_stiffness msg))
    (cl:cons ':q_desired (q_desired msg))
    (cl:cons ':qdot_desired (qdot_desired msg))
    (cl:cons ':q_slew_rate (q_slew_rate msg))
    (cl:cons ':pwm_desired (pwm_desired msg))
    (cl:cons ':enable_motor (enable_motor msg))
))
;//! \htmlinclude M3HumanoidCmd-response.msg.html

(cl:defclass <M3HumanoidCmd-response> (roslisp-msg-protocol:ros-message)
  ((response
    :reader response
    :initarg :response
    :type cl:integer
    :initform 0))
)

(cl:defclass M3HumanoidCmd-response (<M3HumanoidCmd-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <M3HumanoidCmd-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'M3HumanoidCmd-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name m3_client-srv:<M3HumanoidCmd-response> is deprecated: use m3_client-srv:M3HumanoidCmd-response instead.")))

(cl:ensure-generic-function 'response-val :lambda-list '(m))
(cl:defmethod response-val ((m <M3HumanoidCmd-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader m3_client-srv:response-val is deprecated.  Use m3_client-srv:response instead.")
  (response m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <M3HumanoidCmd-response>) ostream)
  "Serializes a message object of type '<M3HumanoidCmd-response>"
  (cl:let* ((signed (cl:slot-value msg 'response)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <M3HumanoidCmd-response>) istream)
  "Deserializes a message object of type '<M3HumanoidCmd-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'response) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<M3HumanoidCmd-response>)))
  "Returns string type for a service object of type '<M3HumanoidCmd-response>"
  "m3_client/M3HumanoidCmdResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'M3HumanoidCmd-response)))
  "Returns string type for a service object of type 'M3HumanoidCmd-response"
  "m3_client/M3HumanoidCmdResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<M3HumanoidCmd-response>)))
  "Returns md5sum for a message object of type '<M3HumanoidCmd-response>"
  "36825d9d1e10e133bf2112ce1e5afc9c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'M3HumanoidCmd-response)))
  "Returns md5sum for a message object of type 'M3HumanoidCmd-response"
  "36825d9d1e10e133bf2112ce1e5afc9c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<M3HumanoidCmd-response>)))
  "Returns full string definition for message of type '<M3HumanoidCmd-response>"
  (cl:format cl:nil "int32 response~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'M3HumanoidCmd-response)))
  "Returns full string definition for message of type 'M3HumanoidCmd-response"
  (cl:format cl:nil "int32 response~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <M3HumanoidCmd-response>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <M3HumanoidCmd-response>))
  "Converts a ROS message object to a list"
  (cl:list 'M3HumanoidCmd-response
    (cl:cons ':response (response msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'M3HumanoidCmd)))
  'M3HumanoidCmd-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'M3HumanoidCmd)))
  'M3HumanoidCmd-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'M3HumanoidCmd)))
  "Returns string type for a service object of type '<M3HumanoidCmd>"
  "m3_client/M3HumanoidCmd")
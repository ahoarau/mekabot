; Auto-generated. Do not edit!


(cl:in-package m3meka_msgs-msg)


;//! \htmlinclude M3JointStatus.msg.html

(cl:defclass <M3JointStatus> (roslisp-msg-protocol:ros-message)
  ((base
    :reader base
    :initarg :base
    :type m3_msgs-msg:M3BaseStatus
    :initform (cl:make-instance 'm3_msgs-msg:M3BaseStatus))
   (motor_temp
    :reader motor_temp
    :initarg :motor_temp
    :type cl:float
    :initform 0.0)
   (amp_temp
    :reader amp_temp
    :initarg :amp_temp
    :type cl:float
    :initform 0.0)
   (current
    :reader current
    :initarg :current
    :type cl:float
    :initform 0.0)
   (torque
    :reader torque
    :initarg :torque
    :type cl:float
    :initform 0.0)
   (torquedot
    :reader torquedot
    :initarg :torquedot
    :type cl:float
    :initform 0.0)
   (theta
    :reader theta
    :initarg :theta
    :type cl:float
    :initform 0.0)
   (thetadot
    :reader thetadot
    :initarg :thetadot
    :type cl:float
    :initform 0.0)
   (thetadotdot
    :reader thetadotdot
    :initarg :thetadotdot
    :type cl:float
    :initform 0.0)
   (torque_gravity
    :reader torque_gravity
    :initarg :torque_gravity
    :type cl:float
    :initform 0.0)
   (pwm_cmd
    :reader pwm_cmd
    :initarg :pwm_cmd
    :type cl:integer
    :initform 0)
   (ambient_temp
    :reader ambient_temp
    :initarg :ambient_temp
    :type cl:float
    :initform 0.0)
   (case_temp
    :reader case_temp
    :initarg :case_temp
    :type cl:float
    :initform 0.0)
   (power
    :reader power
    :initarg :power
    :type cl:float
    :initform 0.0)
   (flags
    :reader flags
    :initarg :flags
    :type cl:integer
    :initform 0))
)

(cl:defclass M3JointStatus (<M3JointStatus>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <M3JointStatus>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'M3JointStatus)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name m3meka_msgs-msg:<M3JointStatus> is deprecated: use m3meka_msgs-msg:M3JointStatus instead.")))

(cl:ensure-generic-function 'base-val :lambda-list '(m))
(cl:defmethod base-val ((m <M3JointStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader m3meka_msgs-msg:base-val is deprecated.  Use m3meka_msgs-msg:base instead.")
  (base m))

(cl:ensure-generic-function 'motor_temp-val :lambda-list '(m))
(cl:defmethod motor_temp-val ((m <M3JointStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader m3meka_msgs-msg:motor_temp-val is deprecated.  Use m3meka_msgs-msg:motor_temp instead.")
  (motor_temp m))

(cl:ensure-generic-function 'amp_temp-val :lambda-list '(m))
(cl:defmethod amp_temp-val ((m <M3JointStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader m3meka_msgs-msg:amp_temp-val is deprecated.  Use m3meka_msgs-msg:amp_temp instead.")
  (amp_temp m))

(cl:ensure-generic-function 'current-val :lambda-list '(m))
(cl:defmethod current-val ((m <M3JointStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader m3meka_msgs-msg:current-val is deprecated.  Use m3meka_msgs-msg:current instead.")
  (current m))

(cl:ensure-generic-function 'torque-val :lambda-list '(m))
(cl:defmethod torque-val ((m <M3JointStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader m3meka_msgs-msg:torque-val is deprecated.  Use m3meka_msgs-msg:torque instead.")
  (torque m))

(cl:ensure-generic-function 'torquedot-val :lambda-list '(m))
(cl:defmethod torquedot-val ((m <M3JointStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader m3meka_msgs-msg:torquedot-val is deprecated.  Use m3meka_msgs-msg:torquedot instead.")
  (torquedot m))

(cl:ensure-generic-function 'theta-val :lambda-list '(m))
(cl:defmethod theta-val ((m <M3JointStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader m3meka_msgs-msg:theta-val is deprecated.  Use m3meka_msgs-msg:theta instead.")
  (theta m))

(cl:ensure-generic-function 'thetadot-val :lambda-list '(m))
(cl:defmethod thetadot-val ((m <M3JointStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader m3meka_msgs-msg:thetadot-val is deprecated.  Use m3meka_msgs-msg:thetadot instead.")
  (thetadot m))

(cl:ensure-generic-function 'thetadotdot-val :lambda-list '(m))
(cl:defmethod thetadotdot-val ((m <M3JointStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader m3meka_msgs-msg:thetadotdot-val is deprecated.  Use m3meka_msgs-msg:thetadotdot instead.")
  (thetadotdot m))

(cl:ensure-generic-function 'torque_gravity-val :lambda-list '(m))
(cl:defmethod torque_gravity-val ((m <M3JointStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader m3meka_msgs-msg:torque_gravity-val is deprecated.  Use m3meka_msgs-msg:torque_gravity instead.")
  (torque_gravity m))

(cl:ensure-generic-function 'pwm_cmd-val :lambda-list '(m))
(cl:defmethod pwm_cmd-val ((m <M3JointStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader m3meka_msgs-msg:pwm_cmd-val is deprecated.  Use m3meka_msgs-msg:pwm_cmd instead.")
  (pwm_cmd m))

(cl:ensure-generic-function 'ambient_temp-val :lambda-list '(m))
(cl:defmethod ambient_temp-val ((m <M3JointStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader m3meka_msgs-msg:ambient_temp-val is deprecated.  Use m3meka_msgs-msg:ambient_temp instead.")
  (ambient_temp m))

(cl:ensure-generic-function 'case_temp-val :lambda-list '(m))
(cl:defmethod case_temp-val ((m <M3JointStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader m3meka_msgs-msg:case_temp-val is deprecated.  Use m3meka_msgs-msg:case_temp instead.")
  (case_temp m))

(cl:ensure-generic-function 'power-val :lambda-list '(m))
(cl:defmethod power-val ((m <M3JointStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader m3meka_msgs-msg:power-val is deprecated.  Use m3meka_msgs-msg:power instead.")
  (power m))

(cl:ensure-generic-function 'flags-val :lambda-list '(m))
(cl:defmethod flags-val ((m <M3JointStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader m3meka_msgs-msg:flags-val is deprecated.  Use m3meka_msgs-msg:flags instead.")
  (flags m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <M3JointStatus>) ostream)
  "Serializes a message object of type '<M3JointStatus>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'base) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'motor_temp))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'amp_temp))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'current))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'torque))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'torquedot))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'theta))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'thetadot))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'thetadotdot))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'torque_gravity))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let* ((signed (cl:slot-value msg 'pwm_cmd)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'ambient_temp))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'case_temp))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'power))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let* ((signed (cl:slot-value msg 'flags)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <M3JointStatus>) istream)
  "Deserializes a message object of type '<M3JointStatus>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'base) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'motor_temp) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'amp_temp) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'current) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'torque) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'torquedot) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'theta) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'thetadot) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'thetadotdot) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'torque_gravity) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'pwm_cmd) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'ambient_temp) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'case_temp) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'power) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'flags) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<M3JointStatus>)))
  "Returns string type for a message object of type '<M3JointStatus>"
  "m3meka_msgs/M3JointStatus")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'M3JointStatus)))
  "Returns string type for a message object of type 'M3JointStatus"
  "m3meka_msgs/M3JointStatus")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<M3JointStatus>)))
  "Returns md5sum for a message object of type '<M3JointStatus>"
  "9c6d93ab28413f8b473c8def3d02284b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'M3JointStatus)))
  "Returns md5sum for a message object of type 'M3JointStatus"
  "9c6d93ab28413f8b473c8def3d02284b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<M3JointStatus>)))
  "Returns full string definition for message of type '<M3JointStatus>"
  (cl:format cl:nil "m3_msgs/M3BaseStatus base~%float32 motor_temp~%float32 amp_temp~%float32 current~%float32 torque~%float32 torquedot~%float32 theta~%float32 thetadot~%float32 thetadotdot~%float32 torque_gravity~%int32 pwm_cmd~%float32 ambient_temp~%float32 case_temp~%float32 power~%int32 flags~%~%================================================================================~%MSG: m3_msgs/M3BaseStatus~%string name~%uint8 state~%int64 timestamp~%string rate~%string version~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'M3JointStatus)))
  "Returns full string definition for message of type 'M3JointStatus"
  (cl:format cl:nil "m3_msgs/M3BaseStatus base~%float32 motor_temp~%float32 amp_temp~%float32 current~%float32 torque~%float32 torquedot~%float32 theta~%float32 thetadot~%float32 thetadotdot~%float32 torque_gravity~%int32 pwm_cmd~%float32 ambient_temp~%float32 case_temp~%float32 power~%int32 flags~%~%================================================================================~%MSG: m3_msgs/M3BaseStatus~%string name~%uint8 state~%int64 timestamp~%string rate~%string version~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <M3JointStatus>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'base))
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <M3JointStatus>))
  "Converts a ROS message object to a list"
  (cl:list 'M3JointStatus
    (cl:cons ':base (base msg))
    (cl:cons ':motor_temp (motor_temp msg))
    (cl:cons ':amp_temp (amp_temp msg))
    (cl:cons ':current (current msg))
    (cl:cons ':torque (torque msg))
    (cl:cons ':torquedot (torquedot msg))
    (cl:cons ':theta (theta msg))
    (cl:cons ':thetadot (thetadot msg))
    (cl:cons ':thetadotdot (thetadotdot msg))
    (cl:cons ':torque_gravity (torque_gravity msg))
    (cl:cons ':pwm_cmd (pwm_cmd msg))
    (cl:cons ':ambient_temp (ambient_temp msg))
    (cl:cons ':case_temp (case_temp msg))
    (cl:cons ':power (power msg))
    (cl:cons ':flags (flags msg))
))

; Auto-generated. Do not edit!


(cl:in-package m3_client-msg)


;//! \htmlinclude M3BaseStatus.msg.html

(cl:defclass <M3BaseStatus> (roslisp-msg-protocol:ros-message)
  ((name
    :reader name
    :initarg :name
    :type cl:string
    :initform "")
   (state
    :reader state
    :initarg :state
    :type cl:fixnum
    :initform 0)
   (timestamp
    :reader timestamp
    :initarg :timestamp
    :type cl:integer
    :initform 0)
   (rate
    :reader rate
    :initarg :rate
    :type cl:string
    :initform "")
   (version
    :reader version
    :initarg :version
    :type cl:string
    :initform ""))
)

(cl:defclass M3BaseStatus (<M3BaseStatus>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <M3BaseStatus>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'M3BaseStatus)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name m3_client-msg:<M3BaseStatus> is deprecated: use m3_client-msg:M3BaseStatus instead.")))

(cl:ensure-generic-function 'name-val :lambda-list '(m))
(cl:defmethod name-val ((m <M3BaseStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader m3_client-msg:name-val is deprecated.  Use m3_client-msg:name instead.")
  (name m))

(cl:ensure-generic-function 'state-val :lambda-list '(m))
(cl:defmethod state-val ((m <M3BaseStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader m3_client-msg:state-val is deprecated.  Use m3_client-msg:state instead.")
  (state m))

(cl:ensure-generic-function 'timestamp-val :lambda-list '(m))
(cl:defmethod timestamp-val ((m <M3BaseStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader m3_client-msg:timestamp-val is deprecated.  Use m3_client-msg:timestamp instead.")
  (timestamp m))

(cl:ensure-generic-function 'rate-val :lambda-list '(m))
(cl:defmethod rate-val ((m <M3BaseStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader m3_client-msg:rate-val is deprecated.  Use m3_client-msg:rate instead.")
  (rate m))

(cl:ensure-generic-function 'version-val :lambda-list '(m))
(cl:defmethod version-val ((m <M3BaseStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader m3_client-msg:version-val is deprecated.  Use m3_client-msg:version instead.")
  (version m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <M3BaseStatus>) ostream)
  "Serializes a message object of type '<M3BaseStatus>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'name))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'state)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'timestamp)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'rate))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'rate))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'version))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'version))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <M3BaseStatus>) istream)
  "Deserializes a message object of type '<M3BaseStatus>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'state)) (cl:read-byte istream))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'timestamp) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'rate) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'rate) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'version) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'version) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<M3BaseStatus>)))
  "Returns string type for a message object of type '<M3BaseStatus>"
  "m3_client/M3BaseStatus")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'M3BaseStatus)))
  "Returns string type for a message object of type 'M3BaseStatus"
  "m3_client/M3BaseStatus")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<M3BaseStatus>)))
  "Returns md5sum for a message object of type '<M3BaseStatus>"
  "7d453ab91423b010362aa62a9d8b20ce")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'M3BaseStatus)))
  "Returns md5sum for a message object of type 'M3BaseStatus"
  "7d453ab91423b010362aa62a9d8b20ce")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<M3BaseStatus>)))
  "Returns full string definition for message of type '<M3BaseStatus>"
  (cl:format cl:nil "string name~%uint8 state~%int64 timestamp~%string rate~%string version~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'M3BaseStatus)))
  "Returns full string definition for message of type 'M3BaseStatus"
  (cl:format cl:nil "string name~%uint8 state~%int64 timestamp~%string rate~%string version~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <M3BaseStatus>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'name))
     1
     8
     4 (cl:length (cl:slot-value msg 'rate))
     4 (cl:length (cl:slot-value msg 'version))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <M3BaseStatus>))
  "Converts a ROS message object to a list"
  (cl:list 'M3BaseStatus
    (cl:cons ':name (name msg))
    (cl:cons ':state (state msg))
    (cl:cons ':timestamp (timestamp msg))
    (cl:cons ':rate (rate msg))
    (cl:cons ':version (version msg))
))

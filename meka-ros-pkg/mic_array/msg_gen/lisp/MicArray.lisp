; Auto-generated. Do not edit!


(cl:in-package mic_array-msg)


;//! \htmlinclude MicArray.msg.html

(cl:defclass <MicArray> (roslisp-msg-protocol:ros-message)
  ((mic_energy
    :reader mic_energy
    :initarg :mic_energy
    :type (cl:vector cl:float)
   :initform (cl:make-array 6 :element-type 'cl:float :initial-element 0.0))
   (angle
    :reader angle
    :initarg :angle
    :type cl:float
    :initform 0.0)
   (mag
    :reader mag
    :initarg :mag
    :type cl:float
    :initform 0.0))
)

(cl:defclass MicArray (<MicArray>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MicArray>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MicArray)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mic_array-msg:<MicArray> is deprecated: use mic_array-msg:MicArray instead.")))

(cl:ensure-generic-function 'mic_energy-val :lambda-list '(m))
(cl:defmethod mic_energy-val ((m <MicArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mic_array-msg:mic_energy-val is deprecated.  Use mic_array-msg:mic_energy instead.")
  (mic_energy m))

(cl:ensure-generic-function 'angle-val :lambda-list '(m))
(cl:defmethod angle-val ((m <MicArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mic_array-msg:angle-val is deprecated.  Use mic_array-msg:angle instead.")
  (angle m))

(cl:ensure-generic-function 'mag-val :lambda-list '(m))
(cl:defmethod mag-val ((m <MicArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mic_array-msg:mag-val is deprecated.  Use mic_array-msg:mag instead.")
  (mag m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MicArray>) ostream)
  "Serializes a message object of type '<MicArray>"
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'mic_energy))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'angle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'mag))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MicArray>) istream)
  "Deserializes a message object of type '<MicArray>"
  (cl:setf (cl:slot-value msg 'mic_energy) (cl:make-array 6))
  (cl:let ((vals (cl:slot-value msg 'mic_energy)))
    (cl:dotimes (i 6)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'angle) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'mag) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MicArray>)))
  "Returns string type for a message object of type '<MicArray>"
  "mic_array/MicArray")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MicArray)))
  "Returns string type for a message object of type 'MicArray"
  "mic_array/MicArray")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MicArray>)))
  "Returns md5sum for a message object of type '<MicArray>"
  "222e1628bdbc23e02ad70bc1470fcd56")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MicArray)))
  "Returns md5sum for a message object of type 'MicArray"
  "222e1628bdbc23e02ad70bc1470fcd56")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MicArray>)))
  "Returns full string definition for message of type '<MicArray>"
  (cl:format cl:nil "float32[6] mic_energy~%float32 angle~%float32 mag~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MicArray)))
  "Returns full string definition for message of type 'MicArray"
  (cl:format cl:nil "float32[6] mic_energy~%float32 angle~%float32 mag~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MicArray>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'mic_energy) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MicArray>))
  "Converts a ROS message object to a list"
  (cl:list 'MicArray
    (cl:cons ':mic_energy (mic_energy msg))
    (cl:cons ':angle (angle msg))
    (cl:cons ':mag (mag msg))
))

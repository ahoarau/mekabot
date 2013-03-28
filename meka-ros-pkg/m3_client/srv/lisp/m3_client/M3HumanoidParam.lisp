; Auto-generated. Do not edit!


(in-package m3_client-srv)


;//! \htmlinclude M3HumanoidParam-request.msg.html

(defclass <M3HumanoidParam-request> (ros-message)
  ((chain
    :reader chain-val
    :initarg :chain
    :type fixnum
    :initform 0)
   (payload_mass
    :reader payload_mass-val
    :initarg :payload_mass
    :type float
    :initform 0.0)
   (payload_com
    :reader payload_com-val
    :initarg :payload_com
    :type (vector float)
   :initform (make-array 3 :element-type 'float :initial-element 0.0))
   (payload_inertia
    :reader payload_inertia-val
    :initarg :payload_inertia
    :type (vector float)
   :initform (make-array 6 :element-type 'float :initial-element 0.0))
   (use_velocities
    :reader use_velocities-val
    :initarg :use_velocities
    :type boolean
    :initform nil)
   (use_accelerations
    :reader use_accelerations-val
    :initarg :use_accelerations
    :type boolean
    :initform nil))
)
(defmethod serialize ((msg <M3HumanoidParam-request>) ostream)
  "Serializes a message object of type '<M3HumanoidParam-request>"
    (write-byte (ldb (byte 8 0) (slot-value msg 'chain)) ostream)
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'payload_mass))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
    (map nil #'(lambda (ele) (let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream)))(slot-value msg 'payload_com))
    (map nil #'(lambda (ele) (let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream)))(slot-value msg 'payload_inertia))
    (write-byte (ldb (byte 8 0) (if (slot-value msg 'use_velocities) 1 0)) ostream)
    (write-byte (ldb (byte 8 0) (if (slot-value msg 'use_accelerations) 1 0)) ostream)
)
(defmethod deserialize ((msg <M3HumanoidParam-request>) istream)
  "Deserializes a message object of type '<M3HumanoidParam-request>"
  (setf (ldb (byte 8 0) (slot-value msg 'chain)) (read-byte istream))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'payload_mass) (roslisp-utils:decode-single-float-bits bits)))
  (setf (slot-value msg 'payload_com) (make-array 3))
  (let ((vals (slot-value msg 'payload_com)))
    (dotimes (i 3)
(let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  (setf (slot-value msg 'payload_inertia) (make-array 6))
  (let ((vals (slot-value msg 'payload_inertia)))
    (dotimes (i 6)
(let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  (setf (slot-value msg 'use_velocities) (not (zerop (read-byte istream))))
  (setf (slot-value msg 'use_accelerations) (not (zerop (read-byte istream))))
  msg
)
(defmethod ros-datatype ((msg (eql '<M3HumanoidParam-request>)))
  "Returns string type for a service object of type '<M3HumanoidParam-request>"
  "m3_client/M3HumanoidParamRequest")
(defmethod md5sum ((type (eql '<M3HumanoidParam-request>)))
  "Returns md5sum for a message object of type '<M3HumanoidParam-request>"
  "34cb417585df77f14f1029d4fc16441a")
(defmethod message-definition ((type (eql '<M3HumanoidParam-request>)))
  "Returns full string definition for message of type '<M3HumanoidParam-request>"
  (format nil "uint8 chain~%float32 payload_mass		#Kg~%float32[3] payload_com		#Meters, End link coordinate system~%float32[6] payload_inertia~%bool use_velocities~%bool use_accelerations~%~%"))
(defmethod serialization-length ((msg <M3HumanoidParam-request>))
  (+ 0
     1
     4
     0 (reduce #'+ (slot-value msg 'payload_com) :key #'(lambda (ele) (declare (ignorable ele)) (+ 4)))
     0 (reduce #'+ (slot-value msg 'payload_inertia) :key #'(lambda (ele) (declare (ignorable ele)) (+ 4)))
     1
     1
))
(defmethod ros-message-to-list ((msg <M3HumanoidParam-request>))
  "Converts a ROS message object to a list"
  (list '<M3HumanoidParam-request>
    (cons ':chain (chain-val msg))
    (cons ':payload_mass (payload_mass-val msg))
    (cons ':payload_com (payload_com-val msg))
    (cons ':payload_inertia (payload_inertia-val msg))
    (cons ':use_velocities (use_velocities-val msg))
    (cons ':use_accelerations (use_accelerations-val msg))
))
;//! \htmlinclude M3HumanoidParam-response.msg.html

(defclass <M3HumanoidParam-response> (ros-message)
  ((response
    :reader response-val
    :initarg :response
    :type integer
    :initform 0))
)
(defmethod serialize ((msg <M3HumanoidParam-response>) ostream)
  "Serializes a message object of type '<M3HumanoidParam-response>"
    (write-byte (ldb (byte 8 0) (slot-value msg 'response)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'response)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'response)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'response)) ostream)
)
(defmethod deserialize ((msg <M3HumanoidParam-response>) istream)
  "Deserializes a message object of type '<M3HumanoidParam-response>"
  (setf (ldb (byte 8 0) (slot-value msg 'response)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'response)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'response)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'response)) (read-byte istream))
  msg
)
(defmethod ros-datatype ((msg (eql '<M3HumanoidParam-response>)))
  "Returns string type for a service object of type '<M3HumanoidParam-response>"
  "m3_client/M3HumanoidParamResponse")
(defmethod md5sum ((type (eql '<M3HumanoidParam-response>)))
  "Returns md5sum for a message object of type '<M3HumanoidParam-response>"
  "34cb417585df77f14f1029d4fc16441a")
(defmethod message-definition ((type (eql '<M3HumanoidParam-response>)))
  "Returns full string definition for message of type '<M3HumanoidParam-response>"
  (format nil "int32 response~%~%"))
(defmethod serialization-length ((msg <M3HumanoidParam-response>))
  (+ 0
     4
))
(defmethod ros-message-to-list ((msg <M3HumanoidParam-response>))
  "Converts a ROS message object to a list"
  (list '<M3HumanoidParam-response>
    (cons ':response (response-val msg))
))
(defmethod service-request-type ((msg (eql 'M3HumanoidParam)))
  '<M3HumanoidParam-request>)
(defmethod service-response-type ((msg (eql 'M3HumanoidParam)))
  '<M3HumanoidParam-response>)
(defmethod ros-datatype ((msg (eql 'M3HumanoidParam)))
  "Returns string type for a service object of type '<M3HumanoidParam>"
  "m3_client/M3HumanoidParam")

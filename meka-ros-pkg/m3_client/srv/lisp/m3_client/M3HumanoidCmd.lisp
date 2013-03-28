; Auto-generated. Do not edit!


(in-package m3_client-srv)


;//! \htmlinclude M3HumanoidCmd-request.msg.html

(defclass <M3HumanoidCmd-request> (ros-message)
  ((chain
    :reader chain-val
    :initarg :chain
    :type fixnum
    :initform 0)
   (tq_desired
    :reader tq_desired-val
    :initarg :tq_desired
    :type (vector float)
   :initform (make-array 0 :element-type 'float :initial-element 0.0))
   (ctrl_mode
    :reader ctrl_mode-val
    :initarg :ctrl_mode
    :type (vector fixnum)
   :initform (make-array 0 :element-type 'fixnum :initial-element 0))
   (q_stiffness
    :reader q_stiffness-val
    :initarg :q_stiffness
    :type (vector float)
   :initform (make-array 0 :element-type 'float :initial-element 0.0))
   (q_desired
    :reader q_desired-val
    :initarg :q_desired
    :type (vector float)
   :initform (make-array 0 :element-type 'float :initial-element 0.0))
   (qdot_desired
    :reader qdot_desired-val
    :initarg :qdot_desired
    :type (vector float)
   :initform (make-array 0 :element-type 'float :initial-element 0.0))
   (q_slew_rate
    :reader q_slew_rate-val
    :initarg :q_slew_rate
    :type (vector float)
   :initform (make-array 0 :element-type 'float :initial-element 0.0))
   (pwm_desired
    :reader pwm_desired-val
    :initarg :pwm_desired
    :type (vector float)
   :initform (make-array 0 :element-type 'float :initial-element 0.0))
   (enable_motor
    :reader enable_motor-val
    :initarg :enable_motor
    :type boolean
    :initform nil))
)
(defmethod serialize ((msg <M3HumanoidCmd-request>) ostream)
  "Serializes a message object of type '<M3HumanoidCmd-request>"
    (write-byte (ldb (byte 8 0) (slot-value msg 'chain)) ostream)
  (let ((__ros_arr_len (length (slot-value msg 'tq_desired))))
    (write-byte (ldb (byte 8 0) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 8) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 16) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 24) __ros_arr_len) ostream))
  (map nil #'(lambda (ele) (let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream)))
    (slot-value msg 'tq_desired))
  (let ((__ros_arr_len (length (slot-value msg 'ctrl_mode))))
    (write-byte (ldb (byte 8 0) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 8) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 16) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 24) __ros_arr_len) ostream))
  (map nil #'(lambda (ele)   (write-byte (ldb (byte 8 0) ele) ostream))
    (slot-value msg 'ctrl_mode))
  (let ((__ros_arr_len (length (slot-value msg 'q_stiffness))))
    (write-byte (ldb (byte 8 0) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 8) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 16) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 24) __ros_arr_len) ostream))
  (map nil #'(lambda (ele) (let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream)))
    (slot-value msg 'q_stiffness))
  (let ((__ros_arr_len (length (slot-value msg 'q_desired))))
    (write-byte (ldb (byte 8 0) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 8) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 16) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 24) __ros_arr_len) ostream))
  (map nil #'(lambda (ele) (let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream)))
    (slot-value msg 'q_desired))
  (let ((__ros_arr_len (length (slot-value msg 'qdot_desired))))
    (write-byte (ldb (byte 8 0) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 8) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 16) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 24) __ros_arr_len) ostream))
  (map nil #'(lambda (ele) (let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream)))
    (slot-value msg 'qdot_desired))
  (let ((__ros_arr_len (length (slot-value msg 'q_slew_rate))))
    (write-byte (ldb (byte 8 0) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 8) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 16) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 24) __ros_arr_len) ostream))
  (map nil #'(lambda (ele) (let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream)))
    (slot-value msg 'q_slew_rate))
  (let ((__ros_arr_len (length (slot-value msg 'pwm_desired))))
    (write-byte (ldb (byte 8 0) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 8) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 16) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 24) __ros_arr_len) ostream))
  (map nil #'(lambda (ele) (let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream)))
    (slot-value msg 'pwm_desired))
    (write-byte (ldb (byte 8 0) (if (slot-value msg 'enable_motor) 1 0)) ostream)
)
(defmethod deserialize ((msg <M3HumanoidCmd-request>) istream)
  "Deserializes a message object of type '<M3HumanoidCmd-request>"
  (setf (ldb (byte 8 0) (slot-value msg 'chain)) (read-byte istream))
  (let ((__ros_arr_len 0))
    (setf (ldb (byte 8 0) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 8) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 16) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 24) __ros_arr_len) (read-byte istream))
    (setf (slot-value msg 'tq_desired) (make-array __ros_arr_len))
    (let ((vals (slot-value msg 'tq_desired)))
      (dotimes (i __ros_arr_len)
(let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  (let ((__ros_arr_len 0))
    (setf (ldb (byte 8 0) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 8) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 16) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 24) __ros_arr_len) (read-byte istream))
    (setf (slot-value msg 'ctrl_mode) (make-array __ros_arr_len))
    (let ((vals (slot-value msg 'ctrl_mode)))
      (dotimes (i __ros_arr_len)
(setf (ldb (byte 8 0) (aref vals i)) (read-byte istream)))))
  (let ((__ros_arr_len 0))
    (setf (ldb (byte 8 0) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 8) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 16) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 24) __ros_arr_len) (read-byte istream))
    (setf (slot-value msg 'q_stiffness) (make-array __ros_arr_len))
    (let ((vals (slot-value msg 'q_stiffness)))
      (dotimes (i __ros_arr_len)
(let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  (let ((__ros_arr_len 0))
    (setf (ldb (byte 8 0) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 8) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 16) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 24) __ros_arr_len) (read-byte istream))
    (setf (slot-value msg 'q_desired) (make-array __ros_arr_len))
    (let ((vals (slot-value msg 'q_desired)))
      (dotimes (i __ros_arr_len)
(let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  (let ((__ros_arr_len 0))
    (setf (ldb (byte 8 0) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 8) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 16) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 24) __ros_arr_len) (read-byte istream))
    (setf (slot-value msg 'qdot_desired) (make-array __ros_arr_len))
    (let ((vals (slot-value msg 'qdot_desired)))
      (dotimes (i __ros_arr_len)
(let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  (let ((__ros_arr_len 0))
    (setf (ldb (byte 8 0) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 8) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 16) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 24) __ros_arr_len) (read-byte istream))
    (setf (slot-value msg 'q_slew_rate) (make-array __ros_arr_len))
    (let ((vals (slot-value msg 'q_slew_rate)))
      (dotimes (i __ros_arr_len)
(let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  (let ((__ros_arr_len 0))
    (setf (ldb (byte 8 0) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 8) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 16) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 24) __ros_arr_len) (read-byte istream))
    (setf (slot-value msg 'pwm_desired) (make-array __ros_arr_len))
    (let ((vals (slot-value msg 'pwm_desired)))
      (dotimes (i __ros_arr_len)
(let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  (setf (slot-value msg 'enable_motor) (not (zerop (read-byte istream))))
  msg
)
(defmethod ros-datatype ((msg (eql '<M3HumanoidCmd-request>)))
  "Returns string type for a service object of type '<M3HumanoidCmd-request>"
  "m3_client/M3HumanoidCmdRequest")
(defmethod md5sum ((type (eql '<M3HumanoidCmd-request>)))
  "Returns md5sum for a message object of type '<M3HumanoidCmd-request>"
  "36825d9d1e10e133bf2112ce1e5afc9c")
(defmethod message-definition ((type (eql '<M3HumanoidCmd-request>)))
  "Returns full string definition for message of type '<M3HumanoidCmd-request>"
  (format nil "uint8 chain~%float32[] tq_desired~%uint8[] ctrl_mode~%float32[] q_stiffness~%float32[] q_desired~%float32[] qdot_desired~%float32[] q_slew_rate~%float32[] pwm_desired~%bool enable_motor~%~%"))
(defmethod serialization-length ((msg <M3HumanoidCmd-request>))
  (+ 0
     1
     4 (reduce #'+ (slot-value msg 'tq_desired) :key #'(lambda (ele) (declare (ignorable ele)) (+ 4)))
     4 (reduce #'+ (slot-value msg 'ctrl_mode) :key #'(lambda (ele) (declare (ignorable ele)) (+ 1)))
     4 (reduce #'+ (slot-value msg 'q_stiffness) :key #'(lambda (ele) (declare (ignorable ele)) (+ 4)))
     4 (reduce #'+ (slot-value msg 'q_desired) :key #'(lambda (ele) (declare (ignorable ele)) (+ 4)))
     4 (reduce #'+ (slot-value msg 'qdot_desired) :key #'(lambda (ele) (declare (ignorable ele)) (+ 4)))
     4 (reduce #'+ (slot-value msg 'q_slew_rate) :key #'(lambda (ele) (declare (ignorable ele)) (+ 4)))
     4 (reduce #'+ (slot-value msg 'pwm_desired) :key #'(lambda (ele) (declare (ignorable ele)) (+ 4)))
     1
))
(defmethod ros-message-to-list ((msg <M3HumanoidCmd-request>))
  "Converts a ROS message object to a list"
  (list '<M3HumanoidCmd-request>
    (cons ':chain (chain-val msg))
    (cons ':tq_desired (tq_desired-val msg))
    (cons ':ctrl_mode (ctrl_mode-val msg))
    (cons ':q_stiffness (q_stiffness-val msg))
    (cons ':q_desired (q_desired-val msg))
    (cons ':qdot_desired (qdot_desired-val msg))
    (cons ':q_slew_rate (q_slew_rate-val msg))
    (cons ':pwm_desired (pwm_desired-val msg))
    (cons ':enable_motor (enable_motor-val msg))
))
;//! \htmlinclude M3HumanoidCmd-response.msg.html

(defclass <M3HumanoidCmd-response> (ros-message)
  ((response
    :reader response-val
    :initarg :response
    :type integer
    :initform 0))
)
(defmethod serialize ((msg <M3HumanoidCmd-response>) ostream)
  "Serializes a message object of type '<M3HumanoidCmd-response>"
    (write-byte (ldb (byte 8 0) (slot-value msg 'response)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'response)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'response)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'response)) ostream)
)
(defmethod deserialize ((msg <M3HumanoidCmd-response>) istream)
  "Deserializes a message object of type '<M3HumanoidCmd-response>"
  (setf (ldb (byte 8 0) (slot-value msg 'response)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'response)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'response)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'response)) (read-byte istream))
  msg
)
(defmethod ros-datatype ((msg (eql '<M3HumanoidCmd-response>)))
  "Returns string type for a service object of type '<M3HumanoidCmd-response>"
  "m3_client/M3HumanoidCmdResponse")
(defmethod md5sum ((type (eql '<M3HumanoidCmd-response>)))
  "Returns md5sum for a message object of type '<M3HumanoidCmd-response>"
  "36825d9d1e10e133bf2112ce1e5afc9c")
(defmethod message-definition ((type (eql '<M3HumanoidCmd-response>)))
  "Returns full string definition for message of type '<M3HumanoidCmd-response>"
  (format nil "int32 response~%~%"))
(defmethod serialization-length ((msg <M3HumanoidCmd-response>))
  (+ 0
     4
))
(defmethod ros-message-to-list ((msg <M3HumanoidCmd-response>))
  "Converts a ROS message object to a list"
  (list '<M3HumanoidCmd-response>
    (cons ':response (response-val msg))
))
(defmethod service-request-type ((msg (eql 'M3HumanoidCmd)))
  '<M3HumanoidCmd-request>)
(defmethod service-response-type ((msg (eql 'M3HumanoidCmd)))
  '<M3HumanoidCmd-response>)
(defmethod ros-datatype ((msg (eql 'M3HumanoidCmd)))
  "Returns string type for a service object of type '<M3HumanoidCmd>"
  "m3_client/M3HumanoidCmd")

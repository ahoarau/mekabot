; Auto-generated. Do not edit!


(in-package m3_client-srv)


;//! \htmlinclude M3LoadX6Status-request.msg.html

(defclass <M3LoadX6Status-request> (ros-message)
  ((request
    :reader request-val
    :initarg :request
    :type integer
    :initform 0))
)
(defmethod serialize ((msg <M3LoadX6Status-request>) ostream)
  "Serializes a message object of type '<M3LoadX6Status-request>"
    (write-byte (ldb (byte 8 0) (slot-value msg 'request)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'request)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'request)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'request)) ostream)
)
(defmethod deserialize ((msg <M3LoadX6Status-request>) istream)
  "Deserializes a message object of type '<M3LoadX6Status-request>"
  (setf (ldb (byte 8 0) (slot-value msg 'request)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'request)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'request)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'request)) (read-byte istream))
  msg
)
(defmethod ros-datatype ((msg (eql '<M3LoadX6Status-request>)))
  "Returns string type for a service object of type '<M3LoadX6Status-request>"
  "m3_client/M3LoadX6StatusRequest")
(defmethod md5sum ((type (eql '<M3LoadX6Status-request>)))
  "Returns md5sum for a message object of type '<M3LoadX6Status-request>"
  "967558453e196f040b12e56469a00597")
(defmethod message-definition ((type (eql '<M3LoadX6Status-request>)))
  "Returns full string definition for message of type '<M3LoadX6Status-request>"
  (format nil "int32 request~%~%"))
(defmethod serialization-length ((msg <M3LoadX6Status-request>))
  (+ 0
     4
))
(defmethod ros-message-to-list ((msg <M3LoadX6Status-request>))
  "Converts a ROS message object to a list"
  (list '<M3LoadX6Status-request>
    (cons ':request (request-val msg))
))
;//! \htmlinclude M3LoadX6Status-response.msg.html

(defclass <M3LoadX6Status-response> (ros-message)
  ((base
    :reader base-val
    :initarg :base
    :type m3_client-msg:<M3BaseStatus>
    :initform (make-instance 'm3_client-msg:<M3BaseStatus>))
   (wrench
    :reader wrench-val
    :initarg :wrench
    :type (vector float)
   :initform (make-array 6 :element-type 'float :initial-element 0.0))
   (adc_ext_0
    :reader adc_ext_0-val
    :initarg :adc_ext_0
    :type float
    :initform 0.0)
   (adc_ext_1
    :reader adc_ext_1-val
    :initarg :adc_ext_1
    :type float
    :initform 0.0)
   (adc_ext_2
    :reader adc_ext_2-val
    :initarg :adc_ext_2
    :type float
    :initform 0.0)
   (dig_ext_0
    :reader dig_ext_0-val
    :initarg :dig_ext_0
    :type float
    :initform 0.0))
)
(defmethod serialize ((msg <M3LoadX6Status-response>) ostream)
  "Serializes a message object of type '<M3LoadX6Status-response>"
  (serialize (slot-value msg 'base) ostream)
    (map nil #'(lambda (ele) (let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream)))(slot-value msg 'wrench))
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'adc_ext_0))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'adc_ext_1))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'adc_ext_2))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'dig_ext_0))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
)
(defmethod deserialize ((msg <M3LoadX6Status-response>) istream)
  "Deserializes a message object of type '<M3LoadX6Status-response>"
  (deserialize (slot-value msg 'base) istream)
  (setf (slot-value msg 'wrench) (make-array 6))
  (let ((vals (slot-value msg 'wrench)))
    (dotimes (i 6)
(let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'adc_ext_0) (roslisp-utils:decode-single-float-bits bits)))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'adc_ext_1) (roslisp-utils:decode-single-float-bits bits)))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'adc_ext_2) (roslisp-utils:decode-single-float-bits bits)))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'dig_ext_0) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(defmethod ros-datatype ((msg (eql '<M3LoadX6Status-response>)))
  "Returns string type for a service object of type '<M3LoadX6Status-response>"
  "m3_client/M3LoadX6StatusResponse")
(defmethod md5sum ((type (eql '<M3LoadX6Status-response>)))
  "Returns md5sum for a message object of type '<M3LoadX6Status-response>"
  "967558453e196f040b12e56469a00597")
(defmethod message-definition ((type (eql '<M3LoadX6Status-response>)))
  "Returns full string definition for message of type '<M3LoadX6Status-response>"
  (format nil "M3BaseStatus base~%float32[6] wrench		#Load cell wrench (mNm)~%float32 adc_ext_0 		~%float32 adc_ext_1 		~%float32 adc_ext_2 		~%float32 dig_ext_0 		~%~%================================================================================~%MSG: m3_client/M3BaseStatus~%string name~%uint8 state~%int64 timestamp~%string rate~%string version~%~%~%~%"))
(defmethod serialization-length ((msg <M3LoadX6Status-response>))
  (+ 0
     (serialization-length (slot-value msg 'base))
     0 (reduce #'+ (slot-value msg 'wrench) :key #'(lambda (ele) (declare (ignorable ele)) (+ 4)))
     4
     4
     4
     4
))
(defmethod ros-message-to-list ((msg <M3LoadX6Status-response>))
  "Converts a ROS message object to a list"
  (list '<M3LoadX6Status-response>
    (cons ':base (base-val msg))
    (cons ':wrench (wrench-val msg))
    (cons ':adc_ext_0 (adc_ext_0-val msg))
    (cons ':adc_ext_1 (adc_ext_1-val msg))
    (cons ':adc_ext_2 (adc_ext_2-val msg))
    (cons ':dig_ext_0 (dig_ext_0-val msg))
))
(defmethod service-request-type ((msg (eql 'M3LoadX6Status)))
  '<M3LoadX6Status-request>)
(defmethod service-response-type ((msg (eql 'M3LoadX6Status)))
  '<M3LoadX6Status-response>)
(defmethod ros-datatype ((msg (eql 'M3LoadX6Status)))
  "Returns string type for a service object of type '<M3LoadX6Status>"
  "m3_client/M3LoadX6Status")

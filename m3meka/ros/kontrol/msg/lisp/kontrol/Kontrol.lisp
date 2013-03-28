; Auto-generated. Do not edit!


(in-package kontrol-msg)


;//! \htmlinclude Kontrol.msg.html

(defclass <Kontrol> (ros-message)
  ((sliders
    :reader sliders-val
    :initarg :sliders
    :type (vector integer)
   :initform (make-array 0 :element-type 'integer :initial-element 0))
   (knobs
    :reader knobs-val
    :initarg :knobs
    :type (vector integer)
   :initform (make-array 0 :element-type 'integer :initial-element 0))
   (buttons
    :reader buttons-val
    :initarg :buttons
    :type (vector integer)
   :initform (make-array 0 :element-type 'integer :initial-element 0)))
)
(defmethod serialize ((msg <Kontrol>) ostream)
  "Serializes a message object of type '<Kontrol>"
  (let ((__ros_arr_len (length (slot-value msg 'sliders))))
    (write-byte (ldb (byte 8 0) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 8) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 16) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 24) __ros_arr_len) ostream))
  (map nil #'(lambda (ele)   (write-byte (ldb (byte 8 0) ele) ostream)
  (write-byte (ldb (byte 8 8) ele) ostream)
  (write-byte (ldb (byte 8 16) ele) ostream)
  (write-byte (ldb (byte 8 24) ele) ostream))
    (slot-value msg 'sliders))
  (let ((__ros_arr_len (length (slot-value msg 'knobs))))
    (write-byte (ldb (byte 8 0) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 8) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 16) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 24) __ros_arr_len) ostream))
  (map nil #'(lambda (ele)   (write-byte (ldb (byte 8 0) ele) ostream)
  (write-byte (ldb (byte 8 8) ele) ostream)
  (write-byte (ldb (byte 8 16) ele) ostream)
  (write-byte (ldb (byte 8 24) ele) ostream))
    (slot-value msg 'knobs))
  (let ((__ros_arr_len (length (slot-value msg 'buttons))))
    (write-byte (ldb (byte 8 0) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 8) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 16) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 24) __ros_arr_len) ostream))
  (map nil #'(lambda (ele)   (write-byte (ldb (byte 8 0) ele) ostream)
  (write-byte (ldb (byte 8 8) ele) ostream)
  (write-byte (ldb (byte 8 16) ele) ostream)
  (write-byte (ldb (byte 8 24) ele) ostream))
    (slot-value msg 'buttons))
)
(defmethod deserialize ((msg <Kontrol>) istream)
  "Deserializes a message object of type '<Kontrol>"
  (let ((__ros_arr_len 0))
    (setf (ldb (byte 8 0) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 8) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 16) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 24) __ros_arr_len) (read-byte istream))
    (setf (slot-value msg 'sliders) (make-array __ros_arr_len))
    (let ((vals (slot-value msg 'sliders)))
      (dotimes (i __ros_arr_len)
(setf (ldb (byte 8 0) (aref vals i)) (read-byte istream))
  (setf (ldb (byte 8 8) (aref vals i)) (read-byte istream))
  (setf (ldb (byte 8 16) (aref vals i)) (read-byte istream))
  (setf (ldb (byte 8 24) (aref vals i)) (read-byte istream)))))
  (let ((__ros_arr_len 0))
    (setf (ldb (byte 8 0) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 8) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 16) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 24) __ros_arr_len) (read-byte istream))
    (setf (slot-value msg 'knobs) (make-array __ros_arr_len))
    (let ((vals (slot-value msg 'knobs)))
      (dotimes (i __ros_arr_len)
(setf (ldb (byte 8 0) (aref vals i)) (read-byte istream))
  (setf (ldb (byte 8 8) (aref vals i)) (read-byte istream))
  (setf (ldb (byte 8 16) (aref vals i)) (read-byte istream))
  (setf (ldb (byte 8 24) (aref vals i)) (read-byte istream)))))
  (let ((__ros_arr_len 0))
    (setf (ldb (byte 8 0) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 8) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 16) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 24) __ros_arr_len) (read-byte istream))
    (setf (slot-value msg 'buttons) (make-array __ros_arr_len))
    (let ((vals (slot-value msg 'buttons)))
      (dotimes (i __ros_arr_len)
(setf (ldb (byte 8 0) (aref vals i)) (read-byte istream))
  (setf (ldb (byte 8 8) (aref vals i)) (read-byte istream))
  (setf (ldb (byte 8 16) (aref vals i)) (read-byte istream))
  (setf (ldb (byte 8 24) (aref vals i)) (read-byte istream)))))
  msg
)
(defmethod ros-datatype ((msg (eql '<Kontrol>)))
  "Returns string type for a message object of type '<Kontrol>"
  "kontrol/Kontrol")
(defmethod md5sum ((type (eql '<Kontrol>)))
  "Returns md5sum for a message object of type '<Kontrol>"
  "c1e4678c7964f4fee41470b640a6873e")
(defmethod message-definition ((type (eql '<Kontrol>)))
  "Returns full string definition for message of type '<Kontrol>"
  (format nil "int32[] sliders~%int32[] knobs~%int32[] buttons~%~%~%~%"))
(defmethod serialization-length ((msg <Kontrol>))
  (+ 0
     4 (reduce #'+ (slot-value msg 'sliders) :key #'(lambda (ele) (declare (ignorable ele)) (+ 4)))
     4 (reduce #'+ (slot-value msg 'knobs) :key #'(lambda (ele) (declare (ignorable ele)) (+ 4)))
     4 (reduce #'+ (slot-value msg 'buttons) :key #'(lambda (ele) (declare (ignorable ele)) (+ 4)))
))
(defmethod ros-message-to-list ((msg <Kontrol>))
  "Converts a ROS message object to a list"
  (list '<Kontrol>
    (cons ':sliders (sliders-val msg))
    (cons ':knobs (knobs-val msg))
    (cons ':buttons (buttons-val msg))
))

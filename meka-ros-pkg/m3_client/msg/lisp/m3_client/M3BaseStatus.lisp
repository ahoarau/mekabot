; Auto-generated. Do not edit!


(in-package m3_client-msg)


;//! \htmlinclude M3BaseStatus.msg.html

(defclass <M3BaseStatus> (ros-message)
  ((name
    :reader name-val
    :initarg :name
    :type string
    :initform "")
   (state
    :reader state-val
    :initarg :state
    :type fixnum
    :initform 0)
   (timestamp
    :reader timestamp-val
    :initarg :timestamp
    :type integer
    :initform 0)
   (rate
    :reader rate-val
    :initarg :rate
    :type string
    :initform "")
   (version
    :reader version-val
    :initarg :version
    :type string
    :initform ""))
)
(defmethod serialize ((msg <M3BaseStatus>) ostream)
  "Serializes a message object of type '<M3BaseStatus>"
  (let ((__ros_str_len (length (slot-value msg 'name))))
    (write-byte (ldb (byte 8 0) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 8) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 16) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 24) __ros_str_len) ostream))
  (map nil #'(lambda (c) (write-byte (char-code c) ostream)) (slot-value msg 'name))
    (write-byte (ldb (byte 8 0) (slot-value msg 'state)) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'timestamp)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'timestamp)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'timestamp)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'timestamp)) ostream)
  (write-byte (ldb (byte 8 32) (slot-value msg 'timestamp)) ostream)
  (write-byte (ldb (byte 8 40) (slot-value msg 'timestamp)) ostream)
  (write-byte (ldb (byte 8 48) (slot-value msg 'timestamp)) ostream)
  (write-byte (ldb (byte 8 56) (slot-value msg 'timestamp)) ostream)
  (let ((__ros_str_len (length (slot-value msg 'rate))))
    (write-byte (ldb (byte 8 0) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 8) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 16) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 24) __ros_str_len) ostream))
  (map nil #'(lambda (c) (write-byte (char-code c) ostream)) (slot-value msg 'rate))
  (let ((__ros_str_len (length (slot-value msg 'version))))
    (write-byte (ldb (byte 8 0) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 8) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 16) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 24) __ros_str_len) ostream))
  (map nil #'(lambda (c) (write-byte (char-code c) ostream)) (slot-value msg 'version))
)
(defmethod deserialize ((msg <M3BaseStatus>) istream)
  "Deserializes a message object of type '<M3BaseStatus>"
  (let ((__ros_str_len 0))
    (setf (ldb (byte 8 0) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 8) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 16) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 24) __ros_str_len) (read-byte istream))
    (setf (slot-value msg 'name) (make-string __ros_str_len))
    (dotimes (__ros_str_idx __ros_str_len msg)
      (setf (char (slot-value msg 'name) __ros_str_idx) (code-char (read-byte istream)))))
  (setf (ldb (byte 8 0) (slot-value msg 'state)) (read-byte istream))
  (setf (ldb (byte 8 0) (slot-value msg 'timestamp)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'timestamp)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'timestamp)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'timestamp)) (read-byte istream))
  (setf (ldb (byte 8 32) (slot-value msg 'timestamp)) (read-byte istream))
  (setf (ldb (byte 8 40) (slot-value msg 'timestamp)) (read-byte istream))
  (setf (ldb (byte 8 48) (slot-value msg 'timestamp)) (read-byte istream))
  (setf (ldb (byte 8 56) (slot-value msg 'timestamp)) (read-byte istream))
  (let ((__ros_str_len 0))
    (setf (ldb (byte 8 0) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 8) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 16) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 24) __ros_str_len) (read-byte istream))
    (setf (slot-value msg 'rate) (make-string __ros_str_len))
    (dotimes (__ros_str_idx __ros_str_len msg)
      (setf (char (slot-value msg 'rate) __ros_str_idx) (code-char (read-byte istream)))))
  (let ((__ros_str_len 0))
    (setf (ldb (byte 8 0) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 8) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 16) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 24) __ros_str_len) (read-byte istream))
    (setf (slot-value msg 'version) (make-string __ros_str_len))
    (dotimes (__ros_str_idx __ros_str_len msg)
      (setf (char (slot-value msg 'version) __ros_str_idx) (code-char (read-byte istream)))))
  msg
)
(defmethod ros-datatype ((msg (eql '<M3BaseStatus>)))
  "Returns string type for a message object of type '<M3BaseStatus>"
  "m3_client/M3BaseStatus")
(defmethod md5sum ((type (eql '<M3BaseStatus>)))
  "Returns md5sum for a message object of type '<M3BaseStatus>"
  "7d453ab91423b010362aa62a9d8b20ce")
(defmethod message-definition ((type (eql '<M3BaseStatus>)))
  "Returns full string definition for message of type '<M3BaseStatus>"
  (format nil "string name~%uint8 state~%int64 timestamp~%string rate~%string version~%~%~%~%"))
(defmethod serialization-length ((msg <M3BaseStatus>))
  (+ 0
     4 (length (slot-value msg 'name))
     1
     8
     4 (length (slot-value msg 'rate))
     4 (length (slot-value msg 'version))
))
(defmethod ros-message-to-list ((msg <M3BaseStatus>))
  "Converts a ROS message object to a list"
  (list '<M3BaseStatus>
    (cons ':name (name-val msg))
    (cons ':state (state-val msg))
    (cons ':timestamp (timestamp-val msg))
    (cons ':rate (rate-val msg))
    (cons ':version (version-val msg))
))

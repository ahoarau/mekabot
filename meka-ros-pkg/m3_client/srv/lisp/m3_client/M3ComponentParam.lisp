; Auto-generated. Do not edit!


(in-package m3_client-srv)


;//! \htmlinclude M3ComponentParam-request.msg.html

(defclass <M3ComponentParam-request> (ros-message)
  ((a
    :reader a-val
    :initarg :a
    :type integer
    :initform 0))
)
(defmethod serialize ((msg <M3ComponentParam-request>) ostream)
  "Serializes a message object of type '<M3ComponentParam-request>"
    (write-byte (ldb (byte 8 0) (slot-value msg 'a)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'a)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'a)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'a)) ostream)
)
(defmethod deserialize ((msg <M3ComponentParam-request>) istream)
  "Deserializes a message object of type '<M3ComponentParam-request>"
  (setf (ldb (byte 8 0) (slot-value msg 'a)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'a)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'a)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'a)) (read-byte istream))
  msg
)
(defmethod ros-datatype ((msg (eql '<M3ComponentParam-request>)))
  "Returns string type for a service object of type '<M3ComponentParam-request>"
  "m3_client/M3ComponentParamRequest")
(defmethod md5sum ((type (eql '<M3ComponentParam-request>)))
  "Returns md5sum for a message object of type '<M3ComponentParam-request>"
  "25ba3fa9d5d930574c4d72dc4151cd60")
(defmethod message-definition ((type (eql '<M3ComponentParam-request>)))
  "Returns full string definition for message of type '<M3ComponentParam-request>"
  (format nil "int32 a~%~%"))
(defmethod serialization-length ((msg <M3ComponentParam-request>))
  (+ 0
     4
))
(defmethod ros-message-to-list ((msg <M3ComponentParam-request>))
  "Converts a ROS message object to a list"
  (list '<M3ComponentParam-request>
    (cons ':a (a-val msg))
))
;//! \htmlinclude M3ComponentParam-response.msg.html

(defclass <M3ComponentParam-response> (ros-message)
  ((b
    :reader b-val
    :initarg :b
    :type integer
    :initform 0))
)
(defmethod serialize ((msg <M3ComponentParam-response>) ostream)
  "Serializes a message object of type '<M3ComponentParam-response>"
    (write-byte (ldb (byte 8 0) (slot-value msg 'b)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'b)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'b)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'b)) ostream)
)
(defmethod deserialize ((msg <M3ComponentParam-response>) istream)
  "Deserializes a message object of type '<M3ComponentParam-response>"
  (setf (ldb (byte 8 0) (slot-value msg 'b)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'b)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'b)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'b)) (read-byte istream))
  msg
)
(defmethod ros-datatype ((msg (eql '<M3ComponentParam-response>)))
  "Returns string type for a service object of type '<M3ComponentParam-response>"
  "m3_client/M3ComponentParamResponse")
(defmethod md5sum ((type (eql '<M3ComponentParam-response>)))
  "Returns md5sum for a message object of type '<M3ComponentParam-response>"
  "25ba3fa9d5d930574c4d72dc4151cd60")
(defmethod message-definition ((type (eql '<M3ComponentParam-response>)))
  "Returns full string definition for message of type '<M3ComponentParam-response>"
  (format nil "int32 b~%~%"))
(defmethod serialization-length ((msg <M3ComponentParam-response>))
  (+ 0
     4
))
(defmethod ros-message-to-list ((msg <M3ComponentParam-response>))
  "Converts a ROS message object to a list"
  (list '<M3ComponentParam-response>
    (cons ':b (b-val msg))
))
(defmethod service-request-type ((msg (eql 'M3ComponentParam)))
  '<M3ComponentParam-request>)
(defmethod service-response-type ((msg (eql 'M3ComponentParam)))
  '<M3ComponentParam-response>)
(defmethod ros-datatype ((msg (eql 'M3ComponentParam)))
  "Returns string type for a service object of type '<M3ComponentParam>"
  "m3_client/M3ComponentParam")

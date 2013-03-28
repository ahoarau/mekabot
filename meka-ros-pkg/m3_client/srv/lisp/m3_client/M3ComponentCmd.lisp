; Auto-generated. Do not edit!


(in-package m3_client-srv)


;//! \htmlinclude M3ComponentCmd-request.msg.html

(defclass <M3ComponentCmd-request> (ros-message)
  ((a
    :reader a-val
    :initarg :a
    :type integer
    :initform 0))
)
(defmethod serialize ((msg <M3ComponentCmd-request>) ostream)
  "Serializes a message object of type '<M3ComponentCmd-request>"
    (write-byte (ldb (byte 8 0) (slot-value msg 'a)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'a)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'a)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'a)) ostream)
)
(defmethod deserialize ((msg <M3ComponentCmd-request>) istream)
  "Deserializes a message object of type '<M3ComponentCmd-request>"
  (setf (ldb (byte 8 0) (slot-value msg 'a)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'a)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'a)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'a)) (read-byte istream))
  msg
)
(defmethod ros-datatype ((msg (eql '<M3ComponentCmd-request>)))
  "Returns string type for a service object of type '<M3ComponentCmd-request>"
  "m3_client/M3ComponentCmdRequest")
(defmethod md5sum ((type (eql '<M3ComponentCmd-request>)))
  "Returns md5sum for a message object of type '<M3ComponentCmd-request>"
  "25ba3fa9d5d930574c4d72dc4151cd60")
(defmethod message-definition ((type (eql '<M3ComponentCmd-request>)))
  "Returns full string definition for message of type '<M3ComponentCmd-request>"
  (format nil "int32 a~%~%"))
(defmethod serialization-length ((msg <M3ComponentCmd-request>))
  (+ 0
     4
))
(defmethod ros-message-to-list ((msg <M3ComponentCmd-request>))
  "Converts a ROS message object to a list"
  (list '<M3ComponentCmd-request>
    (cons ':a (a-val msg))
))
;//! \htmlinclude M3ComponentCmd-response.msg.html

(defclass <M3ComponentCmd-response> (ros-message)
  ((b
    :reader b-val
    :initarg :b
    :type integer
    :initform 0))
)
(defmethod serialize ((msg <M3ComponentCmd-response>) ostream)
  "Serializes a message object of type '<M3ComponentCmd-response>"
    (write-byte (ldb (byte 8 0) (slot-value msg 'b)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'b)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'b)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'b)) ostream)
)
(defmethod deserialize ((msg <M3ComponentCmd-response>) istream)
  "Deserializes a message object of type '<M3ComponentCmd-response>"
  (setf (ldb (byte 8 0) (slot-value msg 'b)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'b)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'b)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'b)) (read-byte istream))
  msg
)
(defmethod ros-datatype ((msg (eql '<M3ComponentCmd-response>)))
  "Returns string type for a service object of type '<M3ComponentCmd-response>"
  "m3_client/M3ComponentCmdResponse")
(defmethod md5sum ((type (eql '<M3ComponentCmd-response>)))
  "Returns md5sum for a message object of type '<M3ComponentCmd-response>"
  "25ba3fa9d5d930574c4d72dc4151cd60")
(defmethod message-definition ((type (eql '<M3ComponentCmd-response>)))
  "Returns full string definition for message of type '<M3ComponentCmd-response>"
  (format nil "int32 b~%~%"))
(defmethod serialization-length ((msg <M3ComponentCmd-response>))
  (+ 0
     4
))
(defmethod ros-message-to-list ((msg <M3ComponentCmd-response>))
  "Converts a ROS message object to a list"
  (list '<M3ComponentCmd-response>
    (cons ':b (b-val msg))
))
(defmethod service-request-type ((msg (eql 'M3ComponentCmd)))
  '<M3ComponentCmd-request>)
(defmethod service-response-type ((msg (eql 'M3ComponentCmd)))
  '<M3ComponentCmd-response>)
(defmethod ros-datatype ((msg (eql 'M3ComponentCmd)))
  "Returns string type for a service object of type '<M3ComponentCmd>"
  "m3_client/M3ComponentCmd")

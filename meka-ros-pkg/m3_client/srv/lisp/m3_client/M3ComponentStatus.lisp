; Auto-generated. Do not edit!


(in-package m3_client-srv)


;//! \htmlinclude M3ComponentStatus-request.msg.html

(defclass <M3ComponentStatus-request> (ros-message)
  ((a
    :reader a-val
    :initarg :a
    :type integer
    :initform 0))
)
(defmethod serialize ((msg <M3ComponentStatus-request>) ostream)
  "Serializes a message object of type '<M3ComponentStatus-request>"
    (write-byte (ldb (byte 8 0) (slot-value msg 'a)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'a)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'a)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'a)) ostream)
)
(defmethod deserialize ((msg <M3ComponentStatus-request>) istream)
  "Deserializes a message object of type '<M3ComponentStatus-request>"
  (setf (ldb (byte 8 0) (slot-value msg 'a)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'a)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'a)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'a)) (read-byte istream))
  msg
)
(defmethod ros-datatype ((msg (eql '<M3ComponentStatus-request>)))
  "Returns string type for a service object of type '<M3ComponentStatus-request>"
  "m3_client/M3ComponentStatusRequest")
(defmethod md5sum ((type (eql '<M3ComponentStatus-request>)))
  "Returns md5sum for a message object of type '<M3ComponentStatus-request>"
  "241529ee0864eca3736d6be302d71b44")
(defmethod message-definition ((type (eql '<M3ComponentStatus-request>)))
  "Returns full string definition for message of type '<M3ComponentStatus-request>"
  (format nil "int32 a~%~%"))
(defmethod serialization-length ((msg <M3ComponentStatus-request>))
  (+ 0
     4
))
(defmethod ros-message-to-list ((msg <M3ComponentStatus-request>))
  "Converts a ROS message object to a list"
  (list '<M3ComponentStatus-request>
    (cons ':a (a-val msg))
))
;//! \htmlinclude M3ComponentStatus-response.msg.html

(defclass <M3ComponentStatus-response> (ros-message)
  ((base
    :reader base-val
    :initarg :base
    :type m3_client-msg:<M3BaseStatus>
    :initform (make-instance 'm3_client-msg:<M3BaseStatus>)))
)
(defmethod serialize ((msg <M3ComponentStatus-response>) ostream)
  "Serializes a message object of type '<M3ComponentStatus-response>"
  (serialize (slot-value msg 'base) ostream)
)
(defmethod deserialize ((msg <M3ComponentStatus-response>) istream)
  "Deserializes a message object of type '<M3ComponentStatus-response>"
  (deserialize (slot-value msg 'base) istream)
  msg
)
(defmethod ros-datatype ((msg (eql '<M3ComponentStatus-response>)))
  "Returns string type for a service object of type '<M3ComponentStatus-response>"
  "m3_client/M3ComponentStatusResponse")
(defmethod md5sum ((type (eql '<M3ComponentStatus-response>)))
  "Returns md5sum for a message object of type '<M3ComponentStatus-response>"
  "241529ee0864eca3736d6be302d71b44")
(defmethod message-definition ((type (eql '<M3ComponentStatus-response>)))
  "Returns full string definition for message of type '<M3ComponentStatus-response>"
  (format nil "M3BaseStatus base~%================================================================================~%MSG: m3_client/M3BaseStatus~%string name~%uint8 state~%int64 timestamp~%string rate~%string version~%~%~%~%"))
(defmethod serialization-length ((msg <M3ComponentStatus-response>))
  (+ 0
     (serialization-length (slot-value msg 'base))
))
(defmethod ros-message-to-list ((msg <M3ComponentStatus-response>))
  "Converts a ROS message object to a list"
  (list '<M3ComponentStatus-response>
    (cons ':base (base-val msg))
))
(defmethod service-request-type ((msg (eql 'M3ComponentStatus)))
  '<M3ComponentStatus-request>)
(defmethod service-response-type ((msg (eql 'M3ComponentStatus)))
  '<M3ComponentStatus-response>)
(defmethod ros-datatype ((msg (eql 'M3ComponentStatus)))
  "Returns string type for a service object of type '<M3ComponentStatus>"
  "m3_client/M3ComponentStatus")

; Auto-generated. Do not edit!


(in-package m3_client-srv)


;//! \htmlinclude M3LoadX6Cmd-request.msg.html

(defclass <M3LoadX6Cmd-request> (ros-message)
  ((request
    :reader request-val
    :initarg :request
    :type integer
    :initform 0))
)
(defmethod serialize ((msg <M3LoadX6Cmd-request>) ostream)
  "Serializes a message object of type '<M3LoadX6Cmd-request>"
    (write-byte (ldb (byte 8 0) (slot-value msg 'request)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'request)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'request)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'request)) ostream)
)
(defmethod deserialize ((msg <M3LoadX6Cmd-request>) istream)
  "Deserializes a message object of type '<M3LoadX6Cmd-request>"
  (setf (ldb (byte 8 0) (slot-value msg 'request)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'request)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'request)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'request)) (read-byte istream))
  msg
)
(defmethod ros-datatype ((msg (eql '<M3LoadX6Cmd-request>)))
  "Returns string type for a service object of type '<M3LoadX6Cmd-request>"
  "m3_client/M3LoadX6CmdRequest")
(defmethod md5sum ((type (eql '<M3LoadX6Cmd-request>)))
  "Returns md5sum for a message object of type '<M3LoadX6Cmd-request>"
  "51edd9dfd50014fde2b589cbf77706aa")
(defmethod message-definition ((type (eql '<M3LoadX6Cmd-request>)))
  "Returns full string definition for message of type '<M3LoadX6Cmd-request>"
  (format nil "int32 request~%~%"))
(defmethod serialization-length ((msg <M3LoadX6Cmd-request>))
  (+ 0
     4
))
(defmethod ros-message-to-list ((msg <M3LoadX6Cmd-request>))
  "Converts a ROS message object to a list"
  (list '<M3LoadX6Cmd-request>
    (cons ':request (request-val msg))
))
;//! \htmlinclude M3LoadX6Cmd-response.msg.html

(defclass <M3LoadX6Cmd-response> (ros-message)
  ((response
    :reader response-val
    :initarg :response
    :type integer
    :initform 0))
)
(defmethod serialize ((msg <M3LoadX6Cmd-response>) ostream)
  "Serializes a message object of type '<M3LoadX6Cmd-response>"
    (write-byte (ldb (byte 8 0) (slot-value msg 'response)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'response)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'response)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'response)) ostream)
)
(defmethod deserialize ((msg <M3LoadX6Cmd-response>) istream)
  "Deserializes a message object of type '<M3LoadX6Cmd-response>"
  (setf (ldb (byte 8 0) (slot-value msg 'response)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'response)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'response)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'response)) (read-byte istream))
  msg
)
(defmethod ros-datatype ((msg (eql '<M3LoadX6Cmd-response>)))
  "Returns string type for a service object of type '<M3LoadX6Cmd-response>"
  "m3_client/M3LoadX6CmdResponse")
(defmethod md5sum ((type (eql '<M3LoadX6Cmd-response>)))
  "Returns md5sum for a message object of type '<M3LoadX6Cmd-response>"
  "51edd9dfd50014fde2b589cbf77706aa")
(defmethod message-definition ((type (eql '<M3LoadX6Cmd-response>)))
  "Returns full string definition for message of type '<M3LoadX6Cmd-response>"
  (format nil "int32 response~%~%"))
(defmethod serialization-length ((msg <M3LoadX6Cmd-response>))
  (+ 0
     4
))
(defmethod ros-message-to-list ((msg <M3LoadX6Cmd-response>))
  "Converts a ROS message object to a list"
  (list '<M3LoadX6Cmd-response>
    (cons ':response (response-val msg))
))
(defmethod service-request-type ((msg (eql 'M3LoadX6Cmd)))
  '<M3LoadX6Cmd-request>)
(defmethod service-response-type ((msg (eql 'M3LoadX6Cmd)))
  '<M3LoadX6Cmd-response>)
(defmethod ros-datatype ((msg (eql 'M3LoadX6Cmd)))
  "Returns string type for a service object of type '<M3LoadX6Cmd>"
  "m3_client/M3LoadX6Cmd")

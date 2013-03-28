; Auto-generated. Do not edit!


(in-package m3_client-srv)


;//! \htmlinclude M3LoadX6Param-request.msg.html

(defclass <M3LoadX6Param-request> (ros-message)
  ((request
    :reader request-val
    :initarg :request
    :type integer
    :initform 0))
)
(defmethod serialize ((msg <M3LoadX6Param-request>) ostream)
  "Serializes a message object of type '<M3LoadX6Param-request>"
    (write-byte (ldb (byte 8 0) (slot-value msg 'request)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'request)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'request)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'request)) ostream)
)
(defmethod deserialize ((msg <M3LoadX6Param-request>) istream)
  "Deserializes a message object of type '<M3LoadX6Param-request>"
  (setf (ldb (byte 8 0) (slot-value msg 'request)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'request)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'request)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'request)) (read-byte istream))
  msg
)
(defmethod ros-datatype ((msg (eql '<M3LoadX6Param-request>)))
  "Returns string type for a service object of type '<M3LoadX6Param-request>"
  "m3_client/M3LoadX6ParamRequest")
(defmethod md5sum ((type (eql '<M3LoadX6Param-request>)))
  "Returns md5sum for a message object of type '<M3LoadX6Param-request>"
  "51edd9dfd50014fde2b589cbf77706aa")
(defmethod message-definition ((type (eql '<M3LoadX6Param-request>)))
  "Returns full string definition for message of type '<M3LoadX6Param-request>"
  (format nil "int32 request~%~%"))
(defmethod serialization-length ((msg <M3LoadX6Param-request>))
  (+ 0
     4
))
(defmethod ros-message-to-list ((msg <M3LoadX6Param-request>))
  "Converts a ROS message object to a list"
  (list '<M3LoadX6Param-request>
    (cons ':request (request-val msg))
))
;//! \htmlinclude M3LoadX6Param-response.msg.html

(defclass <M3LoadX6Param-response> (ros-message)
  ((response
    :reader response-val
    :initarg :response
    :type integer
    :initform 0))
)
(defmethod serialize ((msg <M3LoadX6Param-response>) ostream)
  "Serializes a message object of type '<M3LoadX6Param-response>"
    (write-byte (ldb (byte 8 0) (slot-value msg 'response)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'response)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'response)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'response)) ostream)
)
(defmethod deserialize ((msg <M3LoadX6Param-response>) istream)
  "Deserializes a message object of type '<M3LoadX6Param-response>"
  (setf (ldb (byte 8 0) (slot-value msg 'response)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'response)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'response)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'response)) (read-byte istream))
  msg
)
(defmethod ros-datatype ((msg (eql '<M3LoadX6Param-response>)))
  "Returns string type for a service object of type '<M3LoadX6Param-response>"
  "m3_client/M3LoadX6ParamResponse")
(defmethod md5sum ((type (eql '<M3LoadX6Param-response>)))
  "Returns md5sum for a message object of type '<M3LoadX6Param-response>"
  "51edd9dfd50014fde2b589cbf77706aa")
(defmethod message-definition ((type (eql '<M3LoadX6Param-response>)))
  "Returns full string definition for message of type '<M3LoadX6Param-response>"
  (format nil "int32 response~%~%"))
(defmethod serialization-length ((msg <M3LoadX6Param-response>))
  (+ 0
     4
))
(defmethod ros-message-to-list ((msg <M3LoadX6Param-response>))
  "Converts a ROS message object to a list"
  (list '<M3LoadX6Param-response>
    (cons ':response (response-val msg))
))
(defmethod service-request-type ((msg (eql 'M3LoadX6Param)))
  '<M3LoadX6Param-request>)
(defmethod service-response-type ((msg (eql 'M3LoadX6Param)))
  '<M3LoadX6Param-response>)
(defmethod ros-datatype ((msg (eql 'M3LoadX6Param)))
  "Returns string type for a service object of type '<M3LoadX6Param>"
  "m3_client/M3LoadX6Param")

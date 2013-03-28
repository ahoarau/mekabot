; Auto-generated. Do not edit!


(in-package m3_client-srv)


;//! \htmlinclude M3JointArrayParam-request.msg.html

(defclass <M3JointArrayParam-request> (ros-message)
  ((request
    :reader request-val
    :initarg :request
    :type integer
    :initform 0))
)
(defmethod serialize ((msg <M3JointArrayParam-request>) ostream)
  "Serializes a message object of type '<M3JointArrayParam-request>"
    (write-byte (ldb (byte 8 0) (slot-value msg 'request)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'request)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'request)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'request)) ostream)
)
(defmethod deserialize ((msg <M3JointArrayParam-request>) istream)
  "Deserializes a message object of type '<M3JointArrayParam-request>"
  (setf (ldb (byte 8 0) (slot-value msg 'request)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'request)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'request)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'request)) (read-byte istream))
  msg
)
(defmethod ros-datatype ((msg (eql '<M3JointArrayParam-request>)))
  "Returns string type for a service object of type '<M3JointArrayParam-request>"
  "m3_client/M3JointArrayParamRequest")
(defmethod md5sum ((type (eql '<M3JointArrayParam-request>)))
  "Returns md5sum for a message object of type '<M3JointArrayParam-request>"
  "51edd9dfd50014fde2b589cbf77706aa")
(defmethod message-definition ((type (eql '<M3JointArrayParam-request>)))
  "Returns full string definition for message of type '<M3JointArrayParam-request>"
  (format nil "int32 request~%~%"))
(defmethod serialization-length ((msg <M3JointArrayParam-request>))
  (+ 0
     4
))
(defmethod ros-message-to-list ((msg <M3JointArrayParam-request>))
  "Converts a ROS message object to a list"
  (list '<M3JointArrayParam-request>
    (cons ':request (request-val msg))
))
;//! \htmlinclude M3JointArrayParam-response.msg.html

(defclass <M3JointArrayParam-response> (ros-message)
  ((response
    :reader response-val
    :initarg :response
    :type integer
    :initform 0))
)
(defmethod serialize ((msg <M3JointArrayParam-response>) ostream)
  "Serializes a message object of type '<M3JointArrayParam-response>"
    (write-byte (ldb (byte 8 0) (slot-value msg 'response)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'response)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'response)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'response)) ostream)
)
(defmethod deserialize ((msg <M3JointArrayParam-response>) istream)
  "Deserializes a message object of type '<M3JointArrayParam-response>"
  (setf (ldb (byte 8 0) (slot-value msg 'response)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'response)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'response)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'response)) (read-byte istream))
  msg
)
(defmethod ros-datatype ((msg (eql '<M3JointArrayParam-response>)))
  "Returns string type for a service object of type '<M3JointArrayParam-response>"
  "m3_client/M3JointArrayParamResponse")
(defmethod md5sum ((type (eql '<M3JointArrayParam-response>)))
  "Returns md5sum for a message object of type '<M3JointArrayParam-response>"
  "51edd9dfd50014fde2b589cbf77706aa")
(defmethod message-definition ((type (eql '<M3JointArrayParam-response>)))
  "Returns full string definition for message of type '<M3JointArrayParam-response>"
  (format nil "int32 response~%~%"))
(defmethod serialization-length ((msg <M3JointArrayParam-response>))
  (+ 0
     4
))
(defmethod ros-message-to-list ((msg <M3JointArrayParam-response>))
  "Converts a ROS message object to a list"
  (list '<M3JointArrayParam-response>
    (cons ':response (response-val msg))
))
(defmethod service-request-type ((msg (eql 'M3JointArrayParam)))
  '<M3JointArrayParam-request>)
(defmethod service-response-type ((msg (eql 'M3JointArrayParam)))
  '<M3JointArrayParam-response>)
(defmethod ros-datatype ((msg (eql 'M3JointArrayParam)))
  "Returns string type for a service object of type '<M3JointArrayParam>"
  "m3_client/M3JointArrayParam")

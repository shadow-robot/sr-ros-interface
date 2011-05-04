; Auto-generated. Do not edit!


(in-package sr_edc_ethercat_drivers-srv)


;//! \htmlinclude SimpleMotorFlasher-request.msg.html

(defclass <SimpleMotorFlasher-request> (ros-message)
  ((firmware
    :reader firmware-val
    :initarg :firmware
    :type string
    :initform "")
   (motor_id
    :reader motor_id-val
    :initarg :motor_id
    :type fixnum
    :initform 0))
)
(defmethod serialize ((msg <SimpleMotorFlasher-request>) ostream)
  "Serializes a message object of type '<SimpleMotorFlasher-request>"
  (let ((__ros_str_len (length (slot-value msg 'firmware))))
    (write-byte (ldb (byte 8 0) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 8) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 16) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 24) __ros_str_len) ostream))
  (map nil #'(lambda (c) (write-byte (char-code c) ostream)) (slot-value msg 'firmware))
    (write-byte (ldb (byte 8 0) (slot-value msg 'motor_id)) ostream)
)
(defmethod deserialize ((msg <SimpleMotorFlasher-request>) istream)
  "Deserializes a message object of type '<SimpleMotorFlasher-request>"
  (let ((__ros_str_len 0))
    (setf (ldb (byte 8 0) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 8) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 16) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 24) __ros_str_len) (read-byte istream))
    (setf (slot-value msg 'firmware) (make-string __ros_str_len))
    (dotimes (__ros_str_idx __ros_str_len msg)
      (setf (char (slot-value msg 'firmware) __ros_str_idx) (code-char (read-byte istream)))))
  (setf (ldb (byte 8 0) (slot-value msg 'motor_id)) (read-byte istream))
  msg
)
(defmethod ros-datatype ((msg (eql '<SimpleMotorFlasher-request>)))
  "Returns string type for a service object of type '<SimpleMotorFlasher-request>"
  "sr_edc_ethercat_drivers/SimpleMotorFlasherRequest")
(defmethod md5sum ((type (eql '<SimpleMotorFlasher-request>)))
  "Returns md5sum for a message object of type '<SimpleMotorFlasher-request>"
  "b5e4cd4b44ed54dd8f018caf24a9e0c1")
(defmethod message-definition ((type (eql '<SimpleMotorFlasher-request>)))
  "Returns full string definition for message of type '<SimpleMotorFlasher-request>"
  (format nil "string firmware~%int8 motor_id~%~%"))
(defmethod serialization-length ((msg <SimpleMotorFlasher-request>))
  (+ 0
     4 (length (slot-value msg 'firmware))
     1
))
(defmethod ros-message-to-list ((msg <SimpleMotorFlasher-request>))
  "Converts a ROS message object to a list"
  (list '<SimpleMotorFlasher-request>
    (cons ':firmware (firmware-val msg))
    (cons ':motor_id (motor_id-val msg))
))
;//! \htmlinclude SimpleMotorFlasher-response.msg.html

(defclass <SimpleMotorFlasher-response> (ros-message)
  ((value
    :reader value-val
    :initarg :value
    :type integer
    :initform 0))
)
(defmethod symbol-codes ((msg-type (eql '<SimpleMotorFlasher-response>)))
  "Constants for message type '<SimpleMotorFlasher-response>"
  '((:SUCCESS . 0)
    (:FAIL . 1))
)
(defmethod serialize ((msg <SimpleMotorFlasher-response>) ostream)
  "Serializes a message object of type '<SimpleMotorFlasher-response>"
    (write-byte (ldb (byte 8 0) (slot-value msg 'value)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'value)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'value)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'value)) ostream)
)
(defmethod deserialize ((msg <SimpleMotorFlasher-response>) istream)
  "Deserializes a message object of type '<SimpleMotorFlasher-response>"
  (setf (ldb (byte 8 0) (slot-value msg 'value)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'value)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'value)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'value)) (read-byte istream))
  msg
)
(defmethod ros-datatype ((msg (eql '<SimpleMotorFlasher-response>)))
  "Returns string type for a service object of type '<SimpleMotorFlasher-response>"
  "sr_edc_ethercat_drivers/SimpleMotorFlasherResponse")
(defmethod md5sum ((type (eql '<SimpleMotorFlasher-response>)))
  "Returns md5sum for a message object of type '<SimpleMotorFlasher-response>"
  "b5e4cd4b44ed54dd8f018caf24a9e0c1")
(defmethod message-definition ((type (eql '<SimpleMotorFlasher-response>)))
  "Returns full string definition for message of type '<SimpleMotorFlasher-response>"
  (format nil "int32 value~%# value must take one of the following values~%int32 SUCCESS = 0~%int32 FAIL = 1~%~%~%"))
(defmethod serialization-length ((msg <SimpleMotorFlasher-response>))
  (+ 0
     4
))
(defmethod ros-message-to-list ((msg <SimpleMotorFlasher-response>))
  "Converts a ROS message object to a list"
  (list '<SimpleMotorFlasher-response>
    (cons ':value (value-val msg))
))
(defmethod service-request-type ((msg (eql 'SimpleMotorFlasher)))
  '<SimpleMotorFlasher-request>)
(defmethod service-response-type ((msg (eql 'SimpleMotorFlasher)))
  '<SimpleMotorFlasher-response>)
(defmethod ros-datatype ((msg (eql 'SimpleMotorFlasher)))
  "Returns string type for a service object of type '<SimpleMotorFlasher>"
  "sr_edc_ethercat_drivers/SimpleMotorFlasher")

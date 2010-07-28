; Auto-generated. Do not edit!


(in-package sr_display-srv)


;//! \htmlinclude display_check-request.msg.html

(defclass <display_check-request> (ros-message)
  ((joint_name
    :reader joint_name-val
    :initarg :joint_name
    :type string
    :initform "")
   (attr_name
    :reader attr_name-val
    :initarg :attr_name
    :type string
    :initform "")
   (display
    :reader display-val
    :initarg :display
    :type fixnum
    :initform 0))
)
(defmethod serialize ((msg <display_check-request>) ostream)
  "Serializes a message object of type '<display_check-request>"
  (let ((__ros_str_len (length (slot-value msg 'joint_name))))
    (write-byte (ldb (byte 8 0) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 8) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 16) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 24) __ros_str_len) ostream))
  (map nil #'(lambda (c) (write-byte (char-code c) ostream)) (slot-value msg 'joint_name))
  (let ((__ros_str_len (length (slot-value msg 'attr_name))))
    (write-byte (ldb (byte 8 0) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 8) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 16) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 24) __ros_str_len) ostream))
  (map nil #'(lambda (c) (write-byte (char-code c) ostream)) (slot-value msg 'attr_name))
    (write-byte (ldb (byte 8 0) (slot-value msg 'display)) ostream)
)
(defmethod deserialize ((msg <display_check-request>) istream)
  "Deserializes a message object of type '<display_check-request>"
  (let ((__ros_str_len 0))
    (setf (ldb (byte 8 0) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 8) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 16) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 24) __ros_str_len) (read-byte istream))
    (setf (slot-value msg 'joint_name) (make-string __ros_str_len))
    (dotimes (__ros_str_idx __ros_str_len msg)
      (setf (char (slot-value msg 'joint_name) __ros_str_idx) (code-char (read-byte istream)))))
  (let ((__ros_str_len 0))
    (setf (ldb (byte 8 0) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 8) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 16) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 24) __ros_str_len) (read-byte istream))
    (setf (slot-value msg 'attr_name) (make-string __ros_str_len))
    (dotimes (__ros_str_idx __ros_str_len msg)
      (setf (char (slot-value msg 'attr_name) __ros_str_idx) (code-char (read-byte istream)))))
  (setf (ldb (byte 8 0) (slot-value msg 'display)) (read-byte istream))
  msg
)
(defmethod ros-datatype ((msg (eql '<display_check-request>)))
  "Returns string type for a service object of type '<display_check-request>"
  "sr_display/display_checkRequest")
(defmethod md5sum ((type (eql '<display_check-request>)))
  "Returns md5sum for a message object of type '<display_check-request>"
  "85b2cb070b780fc6900693d47429bb3f")
(defmethod message-definition ((type (eql '<display_check-request>)))
  "Returns full string definition for message of type '<display_check-request>"
  (format nil "string joint_name~%string attr_name~%int8 display~%~%"))
(defmethod serialization-length ((msg <display_check-request>))
  (+ 0
     4 (length (slot-value msg 'joint_name))
     4 (length (slot-value msg 'attr_name))
     1
))
(defmethod ros-message-to-list ((msg <display_check-request>))
  "Converts a ROS message object to a list"
  (list '<display_check-request>
    (cons ':joint_name (joint_name-val msg))
    (cons ':attr_name (attr_name-val msg))
    (cons ':display (display-val msg))
))
;//! \htmlinclude display_check-response.msg.html

(defclass <display_check-response> (ros-message)
  ((success
    :reader success-val
    :initarg :success
    :type fixnum
    :initform 0))
)
(defmethod serialize ((msg <display_check-response>) ostream)
  "Serializes a message object of type '<display_check-response>"
    (write-byte (ldb (byte 8 0) (slot-value msg 'success)) ostream)
)
(defmethod deserialize ((msg <display_check-response>) istream)
  "Deserializes a message object of type '<display_check-response>"
  (setf (ldb (byte 8 0) (slot-value msg 'success)) (read-byte istream))
  msg
)
(defmethod ros-datatype ((msg (eql '<display_check-response>)))
  "Returns string type for a service object of type '<display_check-response>"
  "sr_display/display_checkResponse")
(defmethod md5sum ((type (eql '<display_check-response>)))
  "Returns md5sum for a message object of type '<display_check-response>"
  "85b2cb070b780fc6900693d47429bb3f")
(defmethod message-definition ((type (eql '<display_check-response>)))
  "Returns full string definition for message of type '<display_check-response>"
  (format nil "int8 success~%~%~%"))
(defmethod serialization-length ((msg <display_check-response>))
  (+ 0
     1
))
(defmethod ros-message-to-list ((msg <display_check-response>))
  "Converts a ROS message object to a list"
  (list '<display_check-response>
    (cons ':success (success-val msg))
))
(defmethod service-request-type ((msg (eql 'display_check)))
  '<display_check-request>)
(defmethod service-response-type ((msg (eql 'display_check)))
  '<display_check-response>)
(defmethod ros-datatype ((msg (eql 'display_check)))
  "Returns string type for a service object of type '<display_check>"
  "sr_display/display_check")

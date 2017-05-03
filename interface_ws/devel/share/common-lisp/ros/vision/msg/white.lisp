; Auto-generated. Do not edit!


(cl:in-package vision-msg)


;//! \htmlinclude white.msg.html

(cl:defclass <white> (roslisp-msg-protocol:ros-message)
  ((Gray
    :reader Gray
    :initarg :Gray
    :type cl:integer
    :initform 0)
   (Angle
    :reader Angle
    :initarg :Angle
    :type cl:integer
    :initform 0))
)

(cl:defclass white (<white>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <white>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'white)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name vision-msg:<white> is deprecated: use vision-msg:white instead.")))

(cl:ensure-generic-function 'Gray-val :lambda-list '(m))
(cl:defmethod Gray-val ((m <white>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vision-msg:Gray-val is deprecated.  Use vision-msg:Gray instead.")
  (Gray m))

(cl:ensure-generic-function 'Angle-val :lambda-list '(m))
(cl:defmethod Angle-val ((m <white>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vision-msg:Angle-val is deprecated.  Use vision-msg:Angle instead.")
  (Angle m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <white>) ostream)
  "Serializes a message object of type '<white>"
  (cl:let* ((signed (cl:slot-value msg 'Gray)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'Angle)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <white>) istream)
  "Deserializes a message object of type '<white>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'Gray) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'Angle) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<white>)))
  "Returns string type for a message object of type '<white>"
  "vision/white")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'white)))
  "Returns string type for a message object of type 'white"
  "vision/white")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<white>)))
  "Returns md5sum for a message object of type '<white>"
  "5dea6a3e5a1346d96db1f873a7e99b4e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'white)))
  "Returns md5sum for a message object of type 'white"
  "5dea6a3e5a1346d96db1f873a7e99b4e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<white>)))
  "Returns full string definition for message of type '<white>"
  (cl:format cl:nil "int64 Gray~%int64 Angle~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'white)))
  "Returns full string definition for message of type 'white"
  (cl:format cl:nil "int64 Gray~%int64 Angle~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <white>))
  (cl:+ 0
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <white>))
  "Converts a ROS message object to a list"
  (cl:list 'white
    (cl:cons ':Gray (Gray msg))
    (cl:cons ':Angle (Angle msg))
))

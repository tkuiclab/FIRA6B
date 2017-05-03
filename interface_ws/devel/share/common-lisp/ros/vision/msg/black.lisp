; Auto-generated. Do not edit!


(cl:in-package vision-msg)


;//! \htmlinclude black.msg.html

(cl:defclass <black> (roslisp-msg-protocol:ros-message)
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

(cl:defclass black (<black>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <black>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'black)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name vision-msg:<black> is deprecated: use vision-msg:black instead.")))

(cl:ensure-generic-function 'Gray-val :lambda-list '(m))
(cl:defmethod Gray-val ((m <black>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vision-msg:Gray-val is deprecated.  Use vision-msg:Gray instead.")
  (Gray m))

(cl:ensure-generic-function 'Angle-val :lambda-list '(m))
(cl:defmethod Angle-val ((m <black>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vision-msg:Angle-val is deprecated.  Use vision-msg:Angle instead.")
  (Angle m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <black>) ostream)
  "Serializes a message object of type '<black>"
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <black>) istream)
  "Deserializes a message object of type '<black>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<black>)))
  "Returns string type for a message object of type '<black>"
  "vision/black")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'black)))
  "Returns string type for a message object of type 'black"
  "vision/black")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<black>)))
  "Returns md5sum for a message object of type '<black>"
  "5dea6a3e5a1346d96db1f873a7e99b4e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'black)))
  "Returns md5sum for a message object of type 'black"
  "5dea6a3e5a1346d96db1f873a7e99b4e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<black>)))
  "Returns full string definition for message of type '<black>"
  (cl:format cl:nil "int64 Gray~%int64 Angle~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'black)))
  "Returns full string definition for message of type 'black"
  (cl:format cl:nil "int64 Gray~%int64 Angle~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <black>))
  (cl:+ 0
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <black>))
  "Converts a ROS message object to a list"
  (cl:list 'black
    (cl:cons ':Gray (Gray msg))
    (cl:cons ':Angle (Angle msg))
))

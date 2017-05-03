; Auto-generated. Do not edit!


(cl:in-package vision-msg)


;//! \htmlinclude parametercheck.msg.html

(cl:defclass <parametercheck> (roslisp-msg-protocol:ros-message)
  ((checkpoint
    :reader checkpoint
    :initarg :checkpoint
    :type cl:integer
    :initform 0))
)

(cl:defclass parametercheck (<parametercheck>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <parametercheck>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'parametercheck)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name vision-msg:<parametercheck> is deprecated: use vision-msg:parametercheck instead.")))

(cl:ensure-generic-function 'checkpoint-val :lambda-list '(m))
(cl:defmethod checkpoint-val ((m <parametercheck>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vision-msg:checkpoint-val is deprecated.  Use vision-msg:checkpoint instead.")
  (checkpoint m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <parametercheck>) ostream)
  "Serializes a message object of type '<parametercheck>"
  (cl:let* ((signed (cl:slot-value msg 'checkpoint)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <parametercheck>) istream)
  "Deserializes a message object of type '<parametercheck>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'checkpoint) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<parametercheck>)))
  "Returns string type for a message object of type '<parametercheck>"
  "vision/parametercheck")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'parametercheck)))
  "Returns string type for a message object of type 'parametercheck"
  "vision/parametercheck")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<parametercheck>)))
  "Returns md5sum for a message object of type '<parametercheck>"
  "8d5b7d9ec5b2f1f18019dbf18a79d4c9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'parametercheck)))
  "Returns md5sum for a message object of type 'parametercheck"
  "8d5b7d9ec5b2f1f18019dbf18a79d4c9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<parametercheck>)))
  "Returns full string definition for message of type '<parametercheck>"
  (cl:format cl:nil "int64 checkpoint~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'parametercheck)))
  "Returns full string definition for message of type 'parametercheck"
  (cl:format cl:nil "int64 checkpoint~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <parametercheck>))
  (cl:+ 0
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <parametercheck>))
  "Converts a ROS message object to a list"
  (cl:list 'parametercheck
    (cl:cons ':checkpoint (checkpoint msg))
))

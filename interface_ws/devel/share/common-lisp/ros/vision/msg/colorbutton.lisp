; Auto-generated. Do not edit!


(cl:in-package vision-msg)


;//! \htmlinclude colorbutton.msg.html

(cl:defclass <colorbutton> (roslisp-msg-protocol:ros-message)
  ((button
    :reader button
    :initarg :button
    :type cl:integer
    :initform 0))
)

(cl:defclass colorbutton (<colorbutton>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <colorbutton>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'colorbutton)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name vision-msg:<colorbutton> is deprecated: use vision-msg:colorbutton instead.")))

(cl:ensure-generic-function 'button-val :lambda-list '(m))
(cl:defmethod button-val ((m <colorbutton>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vision-msg:button-val is deprecated.  Use vision-msg:button instead.")
  (button m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <colorbutton>) ostream)
  "Serializes a message object of type '<colorbutton>"
  (cl:let* ((signed (cl:slot-value msg 'button)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <colorbutton>) istream)
  "Deserializes a message object of type '<colorbutton>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'button) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<colorbutton>)))
  "Returns string type for a message object of type '<colorbutton>"
  "vision/colorbutton")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'colorbutton)))
  "Returns string type for a message object of type 'colorbutton"
  "vision/colorbutton")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<colorbutton>)))
  "Returns md5sum for a message object of type '<colorbutton>"
  "636f342c16f994332c9e43675fa80a7c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'colorbutton)))
  "Returns md5sum for a message object of type 'colorbutton"
  "636f342c16f994332c9e43675fa80a7c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<colorbutton>)))
  "Returns full string definition for message of type '<colorbutton>"
  (cl:format cl:nil "int64 button~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'colorbutton)))
  "Returns full string definition for message of type 'colorbutton"
  (cl:format cl:nil "int64 button~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <colorbutton>))
  (cl:+ 0
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <colorbutton>))
  "Converts a ROS message object to a list"
  (cl:list 'colorbutton
    (cl:cons ':button (button msg))
))

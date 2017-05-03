; Auto-generated. Do not edit!


(cl:in-package vision-msg)


;//! \htmlinclude scan.msg.html

(cl:defclass <scan> (roslisp-msg-protocol:ros-message)
  ((Angle_Near_Gap
    :reader Angle_Near_Gap
    :initarg :Angle_Near_Gap
    :type cl:integer
    :initform 0)
   (Magn_Near_Gap
    :reader Magn_Near_Gap
    :initarg :Magn_Near_Gap
    :type cl:integer
    :initform 0)
   (Magn_Near_Start
    :reader Magn_Near_Start
    :initarg :Magn_Near_Start
    :type cl:integer
    :initform 0)
   (Magn_Middle_Start
    :reader Magn_Middle_Start
    :initarg :Magn_Middle_Start
    :type cl:integer
    :initform 0)
   (Magn_Far_Start
    :reader Magn_Far_Start
    :initarg :Magn_Far_Start
    :type cl:integer
    :initform 0)
   (Magn_Far_End
    :reader Magn_Far_End
    :initarg :Magn_Far_End
    :type cl:integer
    :initform 0)
   (Dont_Search_Angle_1
    :reader Dont_Search_Angle_1
    :initarg :Dont_Search_Angle_1
    :type cl:integer
    :initform 0)
   (Dont_Search_Angle_2
    :reader Dont_Search_Angle_2
    :initarg :Dont_Search_Angle_2
    :type cl:integer
    :initform 0)
   (Dont_Search_Angle_3
    :reader Dont_Search_Angle_3
    :initarg :Dont_Search_Angle_3
    :type cl:integer
    :initform 0)
   (Angle_range_1
    :reader Angle_range_1
    :initarg :Angle_range_1
    :type cl:integer
    :initform 0)
   (Angle_range_2_3
    :reader Angle_range_2_3
    :initarg :Angle_range_2_3
    :type cl:integer
    :initform 0))
)

(cl:defclass scan (<scan>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <scan>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'scan)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name vision-msg:<scan> is deprecated: use vision-msg:scan instead.")))

(cl:ensure-generic-function 'Angle_Near_Gap-val :lambda-list '(m))
(cl:defmethod Angle_Near_Gap-val ((m <scan>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vision-msg:Angle_Near_Gap-val is deprecated.  Use vision-msg:Angle_Near_Gap instead.")
  (Angle_Near_Gap m))

(cl:ensure-generic-function 'Magn_Near_Gap-val :lambda-list '(m))
(cl:defmethod Magn_Near_Gap-val ((m <scan>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vision-msg:Magn_Near_Gap-val is deprecated.  Use vision-msg:Magn_Near_Gap instead.")
  (Magn_Near_Gap m))

(cl:ensure-generic-function 'Magn_Near_Start-val :lambda-list '(m))
(cl:defmethod Magn_Near_Start-val ((m <scan>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vision-msg:Magn_Near_Start-val is deprecated.  Use vision-msg:Magn_Near_Start instead.")
  (Magn_Near_Start m))

(cl:ensure-generic-function 'Magn_Middle_Start-val :lambda-list '(m))
(cl:defmethod Magn_Middle_Start-val ((m <scan>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vision-msg:Magn_Middle_Start-val is deprecated.  Use vision-msg:Magn_Middle_Start instead.")
  (Magn_Middle_Start m))

(cl:ensure-generic-function 'Magn_Far_Start-val :lambda-list '(m))
(cl:defmethod Magn_Far_Start-val ((m <scan>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vision-msg:Magn_Far_Start-val is deprecated.  Use vision-msg:Magn_Far_Start instead.")
  (Magn_Far_Start m))

(cl:ensure-generic-function 'Magn_Far_End-val :lambda-list '(m))
(cl:defmethod Magn_Far_End-val ((m <scan>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vision-msg:Magn_Far_End-val is deprecated.  Use vision-msg:Magn_Far_End instead.")
  (Magn_Far_End m))

(cl:ensure-generic-function 'Dont_Search_Angle_1-val :lambda-list '(m))
(cl:defmethod Dont_Search_Angle_1-val ((m <scan>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vision-msg:Dont_Search_Angle_1-val is deprecated.  Use vision-msg:Dont_Search_Angle_1 instead.")
  (Dont_Search_Angle_1 m))

(cl:ensure-generic-function 'Dont_Search_Angle_2-val :lambda-list '(m))
(cl:defmethod Dont_Search_Angle_2-val ((m <scan>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vision-msg:Dont_Search_Angle_2-val is deprecated.  Use vision-msg:Dont_Search_Angle_2 instead.")
  (Dont_Search_Angle_2 m))

(cl:ensure-generic-function 'Dont_Search_Angle_3-val :lambda-list '(m))
(cl:defmethod Dont_Search_Angle_3-val ((m <scan>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vision-msg:Dont_Search_Angle_3-val is deprecated.  Use vision-msg:Dont_Search_Angle_3 instead.")
  (Dont_Search_Angle_3 m))

(cl:ensure-generic-function 'Angle_range_1-val :lambda-list '(m))
(cl:defmethod Angle_range_1-val ((m <scan>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vision-msg:Angle_range_1-val is deprecated.  Use vision-msg:Angle_range_1 instead.")
  (Angle_range_1 m))

(cl:ensure-generic-function 'Angle_range_2_3-val :lambda-list '(m))
(cl:defmethod Angle_range_2_3-val ((m <scan>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vision-msg:Angle_range_2_3-val is deprecated.  Use vision-msg:Angle_range_2_3 instead.")
  (Angle_range_2_3 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <scan>) ostream)
  "Serializes a message object of type '<scan>"
  (cl:let* ((signed (cl:slot-value msg 'Angle_Near_Gap)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'Magn_Near_Gap)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'Magn_Near_Start)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'Magn_Middle_Start)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'Magn_Far_Start)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'Magn_Far_End)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'Dont_Search_Angle_1)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'Dont_Search_Angle_2)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'Dont_Search_Angle_3)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'Angle_range_1)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'Angle_range_2_3)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <scan>) istream)
  "Deserializes a message object of type '<scan>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'Angle_Near_Gap) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'Magn_Near_Gap) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'Magn_Near_Start) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'Magn_Middle_Start) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'Magn_Far_Start) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'Magn_Far_End) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'Dont_Search_Angle_1) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'Dont_Search_Angle_2) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'Dont_Search_Angle_3) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'Angle_range_1) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'Angle_range_2_3) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<scan>)))
  "Returns string type for a message object of type '<scan>"
  "vision/scan")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'scan)))
  "Returns string type for a message object of type 'scan"
  "vision/scan")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<scan>)))
  "Returns md5sum for a message object of type '<scan>"
  "b9e8a289688146cc63d6908befcb947b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'scan)))
  "Returns md5sum for a message object of type 'scan"
  "b9e8a289688146cc63d6908befcb947b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<scan>)))
  "Returns full string definition for message of type '<scan>"
  (cl:format cl:nil "int64 Angle_Near_Gap~%int64 Magn_Near_Gap~%int64 Magn_Near_Start~%int64 Magn_Middle_Start~%int64 Magn_Far_Start~%int64 Magn_Far_End~%int64 Dont_Search_Angle_1~%int64 Dont_Search_Angle_2~%int64 Dont_Search_Angle_3~%int64 Angle_range_1~%int64 Angle_range_2_3~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'scan)))
  "Returns full string definition for message of type 'scan"
  (cl:format cl:nil "int64 Angle_Near_Gap~%int64 Magn_Near_Gap~%int64 Magn_Near_Start~%int64 Magn_Middle_Start~%int64 Magn_Far_Start~%int64 Magn_Far_End~%int64 Dont_Search_Angle_1~%int64 Dont_Search_Angle_2~%int64 Dont_Search_Angle_3~%int64 Angle_range_1~%int64 Angle_range_2_3~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <scan>))
  (cl:+ 0
     8
     8
     8
     8
     8
     8
     8
     8
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <scan>))
  "Converts a ROS message object to a list"
  (cl:list 'scan
    (cl:cons ':Angle_Near_Gap (Angle_Near_Gap msg))
    (cl:cons ':Magn_Near_Gap (Magn_Near_Gap msg))
    (cl:cons ':Magn_Near_Start (Magn_Near_Start msg))
    (cl:cons ':Magn_Middle_Start (Magn_Middle_Start msg))
    (cl:cons ':Magn_Far_Start (Magn_Far_Start msg))
    (cl:cons ':Magn_Far_End (Magn_Far_End msg))
    (cl:cons ':Dont_Search_Angle_1 (Dont_Search_Angle_1 msg))
    (cl:cons ':Dont_Search_Angle_2 (Dont_Search_Angle_2 msg))
    (cl:cons ':Dont_Search_Angle_3 (Dont_Search_Angle_3 msg))
    (cl:cons ':Angle_range_1 (Angle_range_1 msg))
    (cl:cons ':Angle_range_2_3 (Angle_range_2_3 msg))
))

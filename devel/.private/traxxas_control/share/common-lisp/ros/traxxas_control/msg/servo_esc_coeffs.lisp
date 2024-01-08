; Auto-generated. Do not edit!


(cl:in-package traxxas_control-msg)


;//! \htmlinclude servo_esc_coeffs.msg.html

(cl:defclass <servo_esc_coeffs> (roslisp-msg-protocol:ros-message)
  ((servo_neutral
    :reader servo_neutral
    :initarg :servo_neutral
    :type cl:integer
    :initform 0)
   (esc_neutral
    :reader esc_neutral
    :initarg :esc_neutral
    :type cl:integer
    :initform 0)
   (angle_mult
    :reader angle_mult
    :initarg :angle_mult
    :type cl:integer
    :initform 0)
   (throt_mult
    :reader throt_mult
    :initarg :throt_mult
    :type cl:integer
    :initform 0))
)

(cl:defclass servo_esc_coeffs (<servo_esc_coeffs>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <servo_esc_coeffs>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'servo_esc_coeffs)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name traxxas_control-msg:<servo_esc_coeffs> is deprecated: use traxxas_control-msg:servo_esc_coeffs instead.")))

(cl:ensure-generic-function 'servo_neutral-val :lambda-list '(m))
(cl:defmethod servo_neutral-val ((m <servo_esc_coeffs>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader traxxas_control-msg:servo_neutral-val is deprecated.  Use traxxas_control-msg:servo_neutral instead.")
  (servo_neutral m))

(cl:ensure-generic-function 'esc_neutral-val :lambda-list '(m))
(cl:defmethod esc_neutral-val ((m <servo_esc_coeffs>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader traxxas_control-msg:esc_neutral-val is deprecated.  Use traxxas_control-msg:esc_neutral instead.")
  (esc_neutral m))

(cl:ensure-generic-function 'angle_mult-val :lambda-list '(m))
(cl:defmethod angle_mult-val ((m <servo_esc_coeffs>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader traxxas_control-msg:angle_mult-val is deprecated.  Use traxxas_control-msg:angle_mult instead.")
  (angle_mult m))

(cl:ensure-generic-function 'throt_mult-val :lambda-list '(m))
(cl:defmethod throt_mult-val ((m <servo_esc_coeffs>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader traxxas_control-msg:throt_mult-val is deprecated.  Use traxxas_control-msg:throt_mult instead.")
  (throt_mult m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <servo_esc_coeffs>) ostream)
  "Serializes a message object of type '<servo_esc_coeffs>"
  (cl:let* ((signed (cl:slot-value msg 'servo_neutral)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'esc_neutral)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'angle_mult)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'throt_mult)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <servo_esc_coeffs>) istream)
  "Deserializes a message object of type '<servo_esc_coeffs>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'servo_neutral) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'esc_neutral) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'angle_mult) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'throt_mult) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<servo_esc_coeffs>)))
  "Returns string type for a message object of type '<servo_esc_coeffs>"
  "traxxas_control/servo_esc_coeffs")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'servo_esc_coeffs)))
  "Returns string type for a message object of type 'servo_esc_coeffs"
  "traxxas_control/servo_esc_coeffs")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<servo_esc_coeffs>)))
  "Returns md5sum for a message object of type '<servo_esc_coeffs>"
  "2d202238d5f284960838988ffb4a6570")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'servo_esc_coeffs)))
  "Returns md5sum for a message object of type 'servo_esc_coeffs"
  "2d202238d5f284960838988ffb4a6570")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<servo_esc_coeffs>)))
  "Returns full string definition for message of type '<servo_esc_coeffs>"
  (cl:format cl:nil "int32 servo_neutral~%int32 esc_neutral~%~%int32 angle_mult~%int32 throt_mult~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'servo_esc_coeffs)))
  "Returns full string definition for message of type 'servo_esc_coeffs"
  (cl:format cl:nil "int32 servo_neutral~%int32 esc_neutral~%~%int32 angle_mult~%int32 throt_mult~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <servo_esc_coeffs>))
  (cl:+ 0
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <servo_esc_coeffs>))
  "Converts a ROS message object to a list"
  (cl:list 'servo_esc_coeffs
    (cl:cons ':servo_neutral (servo_neutral msg))
    (cl:cons ':esc_neutral (esc_neutral msg))
    (cl:cons ':angle_mult (angle_mult msg))
    (cl:cons ':throt_mult (throt_mult msg))
))

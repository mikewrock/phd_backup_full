; Auto-generated. Do not edit!


(cl:in-package phd-msg)


;//! \htmlinclude marker_val.msg.html

(cl:defclass <marker_val> (roslisp-msg-protocol:ros-message)
  ((i
    :reader i
    :initarg :i
    :type cl:integer
    :initform 0)
   (k
    :reader k
    :initarg :k
    :type cl:integer
    :initform 0)
   (j
    :reader j
    :initarg :j
    :type cl:integer
    :initform 0)
   (val
    :reader val
    :initarg :val
    :type cl:float
    :initform 0.0))
)

(cl:defclass marker_val (<marker_val>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <marker_val>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'marker_val)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name phd-msg:<marker_val> is deprecated: use phd-msg:marker_val instead.")))

(cl:ensure-generic-function 'i-val :lambda-list '(m))
(cl:defmethod i-val ((m <marker_val>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader phd-msg:i-val is deprecated.  Use phd-msg:i instead.")
  (i m))

(cl:ensure-generic-function 'k-val :lambda-list '(m))
(cl:defmethod k-val ((m <marker_val>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader phd-msg:k-val is deprecated.  Use phd-msg:k instead.")
  (k m))

(cl:ensure-generic-function 'j-val :lambda-list '(m))
(cl:defmethod j-val ((m <marker_val>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader phd-msg:j-val is deprecated.  Use phd-msg:j instead.")
  (j m))

(cl:ensure-generic-function 'val-val :lambda-list '(m))
(cl:defmethod val-val ((m <marker_val>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader phd-msg:val-val is deprecated.  Use phd-msg:val instead.")
  (val m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <marker_val>) ostream)
  "Serializes a message object of type '<marker_val>"
  (cl:let* ((signed (cl:slot-value msg 'i)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'k)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'j)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'val))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <marker_val>) istream)
  "Deserializes a message object of type '<marker_val>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'i) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'k) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'j) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'val) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<marker_val>)))
  "Returns string type for a message object of type '<marker_val>"
  "phd/marker_val")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'marker_val)))
  "Returns string type for a message object of type 'marker_val"
  "phd/marker_val")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<marker_val>)))
  "Returns md5sum for a message object of type '<marker_val>"
  "0b968334689a1ff08b48d542e172c4d5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'marker_val)))
  "Returns md5sum for a message object of type 'marker_val"
  "0b968334689a1ff08b48d542e172c4d5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<marker_val>)))
  "Returns full string definition for message of type '<marker_val>"
  (cl:format cl:nil "int32 i~%int32 k~%int32 j~%float32 val~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'marker_val)))
  "Returns full string definition for message of type 'marker_val"
  (cl:format cl:nil "int32 i~%int32 k~%int32 j~%float32 val~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <marker_val>))
  (cl:+ 0
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <marker_val>))
  "Converts a ROS message object to a list"
  (cl:list 'marker_val
    (cl:cons ':i (i msg))
    (cl:cons ':k (k msg))
    (cl:cons ':j (j msg))
    (cl:cons ':val (val msg))
))

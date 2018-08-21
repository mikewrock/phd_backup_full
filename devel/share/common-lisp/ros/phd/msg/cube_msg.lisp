; Auto-generated. Do not edit!


(cl:in-package phd-msg)


;//! \htmlinclude cube_msg.msg.html

(cl:defclass <cube_msg> (roslisp-msg-protocol:ros-message)
  ((j1
    :reader j1
    :initarg :j1
    :type cl:float
    :initform 0.0)
   (vel
    :reader vel
    :initarg :vel
    :type cl:float
    :initform 0.0)
   (acc
    :reader acc
    :initarg :acc
    :type cl:float
    :initform 0.0)
   (pose
    :reader pose
    :initarg :pose
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass cube_msg (<cube_msg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <cube_msg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'cube_msg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name phd-msg:<cube_msg> is deprecated: use phd-msg:cube_msg instead.")))

(cl:ensure-generic-function 'j1-val :lambda-list '(m))
(cl:defmethod j1-val ((m <cube_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader phd-msg:j1-val is deprecated.  Use phd-msg:j1 instead.")
  (j1 m))

(cl:ensure-generic-function 'vel-val :lambda-list '(m))
(cl:defmethod vel-val ((m <cube_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader phd-msg:vel-val is deprecated.  Use phd-msg:vel instead.")
  (vel m))

(cl:ensure-generic-function 'acc-val :lambda-list '(m))
(cl:defmethod acc-val ((m <cube_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader phd-msg:acc-val is deprecated.  Use phd-msg:acc instead.")
  (acc m))

(cl:ensure-generic-function 'pose-val :lambda-list '(m))
(cl:defmethod pose-val ((m <cube_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader phd-msg:pose-val is deprecated.  Use phd-msg:pose instead.")
  (pose m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <cube_msg>) ostream)
  "Serializes a message object of type '<cube_msg>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'j1))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'vel))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'acc))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'pose) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <cube_msg>) istream)
  "Deserializes a message object of type '<cube_msg>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'j1) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'vel) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'acc) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:slot-value msg 'pose) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<cube_msg>)))
  "Returns string type for a message object of type '<cube_msg>"
  "phd/cube_msg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'cube_msg)))
  "Returns string type for a message object of type 'cube_msg"
  "phd/cube_msg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<cube_msg>)))
  "Returns md5sum for a message object of type '<cube_msg>"
  "9433ee31d0089d713563a722639bb582")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'cube_msg)))
  "Returns md5sum for a message object of type 'cube_msg"
  "9433ee31d0089d713563a722639bb582")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<cube_msg>)))
  "Returns full string definition for message of type '<cube_msg>"
  (cl:format cl:nil "float32 j1~%float32 vel~%float32 acc~%bool pose~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'cube_msg)))
  "Returns full string definition for message of type 'cube_msg"
  (cl:format cl:nil "float32 j1~%float32 vel~%float32 acc~%bool pose~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <cube_msg>))
  (cl:+ 0
     4
     4
     4
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <cube_msg>))
  "Converts a ROS message object to a list"
  (cl:list 'cube_msg
    (cl:cons ':j1 (j1 msg))
    (cl:cons ':vel (vel msg))
    (cl:cons ':acc (acc msg))
    (cl:cons ':pose (pose msg))
))

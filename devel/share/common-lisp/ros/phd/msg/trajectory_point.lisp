; Auto-generated. Do not edit!


(cl:in-package phd-msg)


;//! \htmlinclude trajectory_point.msg.html

(cl:defclass <trajectory_point> (roslisp-msg-protocol:ros-message)
  ((x
    :reader x
    :initarg :x
    :type cl:float
    :initform 0.0)
   (y
    :reader y
    :initarg :y
    :type cl:float
    :initform 0.0)
   (z
    :reader z
    :initarg :z
    :type cl:float
    :initform 0.0)
   (nx
    :reader nx
    :initarg :nx
    :type cl:float
    :initform 0.0)
   (ny
    :reader ny
    :initarg :ny
    :type cl:float
    :initform 0.0)
   (nz
    :reader nz
    :initarg :nz
    :type cl:float
    :initform 0.0)
   (d
    :reader d
    :initarg :d
    :type cl:float
    :initform 0.0)
   (d_abs
    :reader d_abs
    :initarg :d_abs
    :type cl:float
    :initform 0.0))
)

(cl:defclass trajectory_point (<trajectory_point>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <trajectory_point>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'trajectory_point)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name phd-msg:<trajectory_point> is deprecated: use phd-msg:trajectory_point instead.")))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <trajectory_point>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader phd-msg:x-val is deprecated.  Use phd-msg:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <trajectory_point>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader phd-msg:y-val is deprecated.  Use phd-msg:y instead.")
  (y m))

(cl:ensure-generic-function 'z-val :lambda-list '(m))
(cl:defmethod z-val ((m <trajectory_point>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader phd-msg:z-val is deprecated.  Use phd-msg:z instead.")
  (z m))

(cl:ensure-generic-function 'nx-val :lambda-list '(m))
(cl:defmethod nx-val ((m <trajectory_point>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader phd-msg:nx-val is deprecated.  Use phd-msg:nx instead.")
  (nx m))

(cl:ensure-generic-function 'ny-val :lambda-list '(m))
(cl:defmethod ny-val ((m <trajectory_point>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader phd-msg:ny-val is deprecated.  Use phd-msg:ny instead.")
  (ny m))

(cl:ensure-generic-function 'nz-val :lambda-list '(m))
(cl:defmethod nz-val ((m <trajectory_point>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader phd-msg:nz-val is deprecated.  Use phd-msg:nz instead.")
  (nz m))

(cl:ensure-generic-function 'd-val :lambda-list '(m))
(cl:defmethod d-val ((m <trajectory_point>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader phd-msg:d-val is deprecated.  Use phd-msg:d instead.")
  (d m))

(cl:ensure-generic-function 'd_abs-val :lambda-list '(m))
(cl:defmethod d_abs-val ((m <trajectory_point>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader phd-msg:d_abs-val is deprecated.  Use phd-msg:d_abs instead.")
  (d_abs m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <trajectory_point>) ostream)
  "Serializes a message object of type '<trajectory_point>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'z))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'nx))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'ny))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'nz))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'd))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'd_abs))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <trajectory_point>) istream)
  "Deserializes a message object of type '<trajectory_point>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'z) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'nx) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'ny) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'nz) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'd) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'd_abs) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<trajectory_point>)))
  "Returns string type for a message object of type '<trajectory_point>"
  "phd/trajectory_point")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'trajectory_point)))
  "Returns string type for a message object of type 'trajectory_point"
  "phd/trajectory_point")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<trajectory_point>)))
  "Returns md5sum for a message object of type '<trajectory_point>"
  "fa2a935c0849c33dde0d2d520826cfdb")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'trajectory_point)))
  "Returns md5sum for a message object of type 'trajectory_point"
  "fa2a935c0849c33dde0d2d520826cfdb")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<trajectory_point>)))
  "Returns full string definition for message of type '<trajectory_point>"
  (cl:format cl:nil "float32 x~%float32 y~%float32 z~%float32 nx~%float32 ny~%float32 nz~%float32 d~%float32 d_abs~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'trajectory_point)))
  "Returns full string definition for message of type 'trajectory_point"
  (cl:format cl:nil "float32 x~%float32 y~%float32 z~%float32 nx~%float32 ny~%float32 nz~%float32 d~%float32 d_abs~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <trajectory_point>))
  (cl:+ 0
     4
     4
     4
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <trajectory_point>))
  "Converts a ROS message object to a list"
  (cl:list 'trajectory_point
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
    (cl:cons ':z (z msg))
    (cl:cons ':nx (nx msg))
    (cl:cons ':ny (ny msg))
    (cl:cons ':nz (nz msg))
    (cl:cons ':d (d msg))
    (cl:cons ':d_abs (d_abs msg))
))

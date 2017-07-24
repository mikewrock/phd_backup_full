; Auto-generated. Do not edit!


(cl:in-package wrock-msg)


;//! \htmlinclude cube_pose.msg.html

(cl:defclass <cube_pose> (roslisp-msg-protocol:ros-message)
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
   (pitch
    :reader pitch
    :initarg :pitch
    :type cl:float
    :initform 0.0)
   (yaw
    :reader yaw
    :initarg :yaw
    :type cl:float
    :initform 0.0)
   (xrot
    :reader xrot
    :initarg :xrot
    :type cl:float
    :initform 0.0)
   (yrot
    :reader yrot
    :initarg :yrot
    :type cl:float
    :initform 0.0)
   (zrot
    :reader zrot
    :initarg :zrot
    :type cl:float
    :initform 0.0))
)

(cl:defclass cube_pose (<cube_pose>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <cube_pose>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'cube_pose)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name wrock-msg:<cube_pose> is deprecated: use wrock-msg:cube_pose instead.")))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <cube_pose>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader wrock-msg:x-val is deprecated.  Use wrock-msg:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <cube_pose>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader wrock-msg:y-val is deprecated.  Use wrock-msg:y instead.")
  (y m))

(cl:ensure-generic-function 'z-val :lambda-list '(m))
(cl:defmethod z-val ((m <cube_pose>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader wrock-msg:z-val is deprecated.  Use wrock-msg:z instead.")
  (z m))

(cl:ensure-generic-function 'pitch-val :lambda-list '(m))
(cl:defmethod pitch-val ((m <cube_pose>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader wrock-msg:pitch-val is deprecated.  Use wrock-msg:pitch instead.")
  (pitch m))

(cl:ensure-generic-function 'yaw-val :lambda-list '(m))
(cl:defmethod yaw-val ((m <cube_pose>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader wrock-msg:yaw-val is deprecated.  Use wrock-msg:yaw instead.")
  (yaw m))

(cl:ensure-generic-function 'xrot-val :lambda-list '(m))
(cl:defmethod xrot-val ((m <cube_pose>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader wrock-msg:xrot-val is deprecated.  Use wrock-msg:xrot instead.")
  (xrot m))

(cl:ensure-generic-function 'yrot-val :lambda-list '(m))
(cl:defmethod yrot-val ((m <cube_pose>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader wrock-msg:yrot-val is deprecated.  Use wrock-msg:yrot instead.")
  (yrot m))

(cl:ensure-generic-function 'zrot-val :lambda-list '(m))
(cl:defmethod zrot-val ((m <cube_pose>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader wrock-msg:zrot-val is deprecated.  Use wrock-msg:zrot instead.")
  (zrot m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <cube_pose>) ostream)
  "Serializes a message object of type '<cube_pose>"
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
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'pitch))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'yaw))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'xrot))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'yrot))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'zrot))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <cube_pose>) istream)
  "Deserializes a message object of type '<cube_pose>"
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
    (cl:setf (cl:slot-value msg 'pitch) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'yaw) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'xrot) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'yrot) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'zrot) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<cube_pose>)))
  "Returns string type for a message object of type '<cube_pose>"
  "wrock/cube_pose")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'cube_pose)))
  "Returns string type for a message object of type 'cube_pose"
  "wrock/cube_pose")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<cube_pose>)))
  "Returns md5sum for a message object of type '<cube_pose>"
  "9b97735605d3278f4692f5c037707560")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'cube_pose)))
  "Returns md5sum for a message object of type 'cube_pose"
  "9b97735605d3278f4692f5c037707560")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<cube_pose>)))
  "Returns full string definition for message of type '<cube_pose>"
  (cl:format cl:nil "float32 x~%float32 y~%float32 z~%float32 pitch~%float32 yaw~%float32 xrot~%float32 yrot~%float32 zrot~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'cube_pose)))
  "Returns full string definition for message of type 'cube_pose"
  (cl:format cl:nil "float32 x~%float32 y~%float32 z~%float32 pitch~%float32 yaw~%float32 xrot~%float32 yrot~%float32 zrot~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <cube_pose>))
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
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <cube_pose>))
  "Converts a ROS message object to a list"
  (cl:list 'cube_pose
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
    (cl:cons ':z (z msg))
    (cl:cons ':pitch (pitch msg))
    (cl:cons ':yaw (yaw msg))
    (cl:cons ':xrot (xrot msg))
    (cl:cons ':yrot (yrot msg))
    (cl:cons ':zrot (zrot msg))
))

; Auto-generated. Do not edit!


(cl:in-package wrock-msg)


;//! \htmlinclude joint_pose.msg.html

(cl:defclass <joint_pose> (roslisp-msg-protocol:ros-message)
  ((j1
    :reader j1
    :initarg :j1
    :type cl:float
    :initform 0.0)
   (j2
    :reader j2
    :initarg :j2
    :type cl:float
    :initform 0.0)
   (j3
    :reader j3
    :initarg :j3
    :type cl:float
    :initform 0.0)
   (j4
    :reader j4
    :initarg :j4
    :type cl:float
    :initform 0.0)
   (j5
    :reader j5
    :initarg :j5
    :type cl:float
    :initform 0.0))
)

(cl:defclass joint_pose (<joint_pose>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <joint_pose>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'joint_pose)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name wrock-msg:<joint_pose> is deprecated: use wrock-msg:joint_pose instead.")))

(cl:ensure-generic-function 'j1-val :lambda-list '(m))
(cl:defmethod j1-val ((m <joint_pose>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader wrock-msg:j1-val is deprecated.  Use wrock-msg:j1 instead.")
  (j1 m))

(cl:ensure-generic-function 'j2-val :lambda-list '(m))
(cl:defmethod j2-val ((m <joint_pose>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader wrock-msg:j2-val is deprecated.  Use wrock-msg:j2 instead.")
  (j2 m))

(cl:ensure-generic-function 'j3-val :lambda-list '(m))
(cl:defmethod j3-val ((m <joint_pose>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader wrock-msg:j3-val is deprecated.  Use wrock-msg:j3 instead.")
  (j3 m))

(cl:ensure-generic-function 'j4-val :lambda-list '(m))
(cl:defmethod j4-val ((m <joint_pose>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader wrock-msg:j4-val is deprecated.  Use wrock-msg:j4 instead.")
  (j4 m))

(cl:ensure-generic-function 'j5-val :lambda-list '(m))
(cl:defmethod j5-val ((m <joint_pose>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader wrock-msg:j5-val is deprecated.  Use wrock-msg:j5 instead.")
  (j5 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <joint_pose>) ostream)
  "Serializes a message object of type '<joint_pose>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'j1))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'j2))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'j3))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'j4))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'j5))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <joint_pose>) istream)
  "Deserializes a message object of type '<joint_pose>"
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
    (cl:setf (cl:slot-value msg 'j2) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'j3) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'j4) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'j5) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<joint_pose>)))
  "Returns string type for a message object of type '<joint_pose>"
  "wrock/joint_pose")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'joint_pose)))
  "Returns string type for a message object of type 'joint_pose"
  "wrock/joint_pose")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<joint_pose>)))
  "Returns md5sum for a message object of type '<joint_pose>"
  "c0bd5c4d3ba9c1c1ae78ffd71bd90da0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'joint_pose)))
  "Returns md5sum for a message object of type 'joint_pose"
  "c0bd5c4d3ba9c1c1ae78ffd71bd90da0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<joint_pose>)))
  "Returns full string definition for message of type '<joint_pose>"
  (cl:format cl:nil "float32 j1~%float32 j2~%float32 j3~%float32 j4~%float32 j5~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'joint_pose)))
  "Returns full string definition for message of type 'joint_pose"
  (cl:format cl:nil "float32 j1~%float32 j2~%float32 j3~%float32 j4~%float32 j5~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <joint_pose>))
  (cl:+ 0
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <joint_pose>))
  "Converts a ROS message object to a list"
  (cl:list 'joint_pose
    (cl:cons ':j1 (j1 msg))
    (cl:cons ':j2 (j2 msg))
    (cl:cons ':j3 (j3 msg))
    (cl:cons ':j4 (j4 msg))
    (cl:cons ':j5 (j5 msg))
))

; Auto-generated. Do not edit!


(cl:in-package phd-msg)


;//! \htmlinclude marker_msg.msg.html

(cl:defclass <marker_msg> (roslisp-msg-protocol:ros-message)
  ((p1
    :reader p1
    :initarg :p1
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (p2
    :reader p2
    :initarg :p2
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (p3
    :reader p3
    :initarg :p3
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (vec1
    :reader vec1
    :initarg :vec1
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (vec2
    :reader vec2
    :initarg :vec2
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (transform
    :reader transform
    :initarg :transform
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (VAL1
    :reader VAL1
    :initarg :VAL1
    :type cl:float
    :initform 0.0)
   (VAL2
    :reader VAL2
    :initarg :VAL2
    :type cl:float
    :initform 0.0)
   (VAL3
    :reader VAL3
    :initarg :VAL3
    :type cl:float
    :initform 0.0)
   (VAL4
    :reader VAL4
    :initarg :VAL4
    :type cl:float
    :initform 0.0))
)

(cl:defclass marker_msg (<marker_msg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <marker_msg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'marker_msg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name phd-msg:<marker_msg> is deprecated: use phd-msg:marker_msg instead.")))

(cl:ensure-generic-function 'p1-val :lambda-list '(m))
(cl:defmethod p1-val ((m <marker_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader phd-msg:p1-val is deprecated.  Use phd-msg:p1 instead.")
  (p1 m))

(cl:ensure-generic-function 'p2-val :lambda-list '(m))
(cl:defmethod p2-val ((m <marker_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader phd-msg:p2-val is deprecated.  Use phd-msg:p2 instead.")
  (p2 m))

(cl:ensure-generic-function 'p3-val :lambda-list '(m))
(cl:defmethod p3-val ((m <marker_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader phd-msg:p3-val is deprecated.  Use phd-msg:p3 instead.")
  (p3 m))

(cl:ensure-generic-function 'vec1-val :lambda-list '(m))
(cl:defmethod vec1-val ((m <marker_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader phd-msg:vec1-val is deprecated.  Use phd-msg:vec1 instead.")
  (vec1 m))

(cl:ensure-generic-function 'vec2-val :lambda-list '(m))
(cl:defmethod vec2-val ((m <marker_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader phd-msg:vec2-val is deprecated.  Use phd-msg:vec2 instead.")
  (vec2 m))

(cl:ensure-generic-function 'transform-val :lambda-list '(m))
(cl:defmethod transform-val ((m <marker_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader phd-msg:transform-val is deprecated.  Use phd-msg:transform instead.")
  (transform m))

(cl:ensure-generic-function 'VAL1-val :lambda-list '(m))
(cl:defmethod VAL1-val ((m <marker_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader phd-msg:VAL1-val is deprecated.  Use phd-msg:VAL1 instead.")
  (VAL1 m))

(cl:ensure-generic-function 'VAL2-val :lambda-list '(m))
(cl:defmethod VAL2-val ((m <marker_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader phd-msg:VAL2-val is deprecated.  Use phd-msg:VAL2 instead.")
  (VAL2 m))

(cl:ensure-generic-function 'VAL3-val :lambda-list '(m))
(cl:defmethod VAL3-val ((m <marker_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader phd-msg:VAL3-val is deprecated.  Use phd-msg:VAL3 instead.")
  (VAL3 m))

(cl:ensure-generic-function 'VAL4-val :lambda-list '(m))
(cl:defmethod VAL4-val ((m <marker_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader phd-msg:VAL4-val is deprecated.  Use phd-msg:VAL4 instead.")
  (VAL4 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <marker_msg>) ostream)
  "Serializes a message object of type '<marker_msg>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'p1) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'p2) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'p3) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'vec1) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'vec2) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'transform))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'transform))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'VAL1))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'VAL2))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'VAL3))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'VAL4))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <marker_msg>) istream)
  "Deserializes a message object of type '<marker_msg>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'p1) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'p2) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'p3) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'vec1) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'vec2) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'transform) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'transform)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'VAL1) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'VAL2) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'VAL3) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'VAL4) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<marker_msg>)))
  "Returns string type for a message object of type '<marker_msg>"
  "phd/marker_msg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'marker_msg)))
  "Returns string type for a message object of type 'marker_msg"
  "phd/marker_msg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<marker_msg>)))
  "Returns md5sum for a message object of type '<marker_msg>"
  "46163b1a11ff6053bb2e316cbd0d9ada")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'marker_msg)))
  "Returns md5sum for a message object of type 'marker_msg"
  "46163b1a11ff6053bb2e316cbd0d9ada")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<marker_msg>)))
  "Returns full string definition for message of type '<marker_msg>"
  (cl:format cl:nil "geometry_msgs/Point p1~%geometry_msgs/Point p2~%geometry_msgs/Point p3~%geometry_msgs/Point vec1~%geometry_msgs/Point vec2~%float64[] transform~%float32 VAL1~%float32 VAL2~%float32 VAL3~%float32 VAL4~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'marker_msg)))
  "Returns full string definition for message of type 'marker_msg"
  (cl:format cl:nil "geometry_msgs/Point p1~%geometry_msgs/Point p2~%geometry_msgs/Point p3~%geometry_msgs/Point vec1~%geometry_msgs/Point vec2~%float64[] transform~%float32 VAL1~%float32 VAL2~%float32 VAL3~%float32 VAL4~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <marker_msg>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'p1))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'p2))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'p3))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'vec1))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'vec2))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'transform) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <marker_msg>))
  "Converts a ROS message object to a list"
  (cl:list 'marker_msg
    (cl:cons ':p1 (p1 msg))
    (cl:cons ':p2 (p2 msg))
    (cl:cons ':p3 (p3 msg))
    (cl:cons ':vec1 (vec1 msg))
    (cl:cons ':vec2 (vec2 msg))
    (cl:cons ':transform (transform msg))
    (cl:cons ':VAL1 (VAL1 msg))
    (cl:cons ':VAL2 (VAL2 msg))
    (cl:cons ':VAL3 (VAL3 msg))
    (cl:cons ':VAL4 (VAL4 msg))
))

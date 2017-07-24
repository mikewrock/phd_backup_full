; Auto-generated. Do not edit!


(cl:in-package phd-msg)


;//! \htmlinclude trajectory_section.msg.html

(cl:defclass <trajectory_section> (roslisp-msg-protocol:ros-message)
  ((points
    :reader points
    :initarg :points
    :type (cl:vector phd-msg:trajectory_point)
   :initform (cl:make-array 0 :element-type 'phd-msg:trajectory_point :initial-element (cl:make-instance 'phd-msg:trajectory_point)))
   (end_point
    :reader end_point
    :initarg :end_point
    :type phd-msg:trajectory_point
    :initform (cl:make-instance 'phd-msg:trajectory_point))
   (start_point
    :reader start_point
    :initarg :start_point
    :type phd-msg:trajectory_point
    :initform (cl:make-instance 'phd-msg:trajectory_point))
   (z_height
    :reader z_height
    :initarg :z_height
    :type cl:float
    :initform 0.0))
)

(cl:defclass trajectory_section (<trajectory_section>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <trajectory_section>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'trajectory_section)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name phd-msg:<trajectory_section> is deprecated: use phd-msg:trajectory_section instead.")))

(cl:ensure-generic-function 'points-val :lambda-list '(m))
(cl:defmethod points-val ((m <trajectory_section>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader phd-msg:points-val is deprecated.  Use phd-msg:points instead.")
  (points m))

(cl:ensure-generic-function 'end_point-val :lambda-list '(m))
(cl:defmethod end_point-val ((m <trajectory_section>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader phd-msg:end_point-val is deprecated.  Use phd-msg:end_point instead.")
  (end_point m))

(cl:ensure-generic-function 'start_point-val :lambda-list '(m))
(cl:defmethod start_point-val ((m <trajectory_section>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader phd-msg:start_point-val is deprecated.  Use phd-msg:start_point instead.")
  (start_point m))

(cl:ensure-generic-function 'z_height-val :lambda-list '(m))
(cl:defmethod z_height-val ((m <trajectory_section>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader phd-msg:z_height-val is deprecated.  Use phd-msg:z_height instead.")
  (z_height m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <trajectory_section>) ostream)
  "Serializes a message object of type '<trajectory_section>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'points))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'points))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'end_point) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'start_point) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'z_height))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <trajectory_section>) istream)
  "Deserializes a message object of type '<trajectory_section>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'points) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'points)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'phd-msg:trajectory_point))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'end_point) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'start_point) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'z_height) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<trajectory_section>)))
  "Returns string type for a message object of type '<trajectory_section>"
  "phd/trajectory_section")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'trajectory_section)))
  "Returns string type for a message object of type 'trajectory_section"
  "phd/trajectory_section")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<trajectory_section>)))
  "Returns md5sum for a message object of type '<trajectory_section>"
  "79d9ff5c72f5660fbbe2563aa76fd1c1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'trajectory_section)))
  "Returns md5sum for a message object of type 'trajectory_section"
  "79d9ff5c72f5660fbbe2563aa76fd1c1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<trajectory_section>)))
  "Returns full string definition for message of type '<trajectory_section>"
  (cl:format cl:nil "trajectory_point[] points~%trajectory_point end_point~%trajectory_point start_point~%float32 z_height~%~%~%~%================================================================================~%MSG: phd/trajectory_point~%float32 x~%float32 y~%float32 z~%float32 nx~%float32 ny~%float32 nz~%float32 d~%float32 d_abs~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'trajectory_section)))
  "Returns full string definition for message of type 'trajectory_section"
  (cl:format cl:nil "trajectory_point[] points~%trajectory_point end_point~%trajectory_point start_point~%float32 z_height~%~%~%~%================================================================================~%MSG: phd/trajectory_point~%float32 x~%float32 y~%float32 z~%float32 nx~%float32 ny~%float32 nz~%float32 d~%float32 d_abs~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <trajectory_section>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'points) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'end_point))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'start_point))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <trajectory_section>))
  "Converts a ROS message object to a list"
  (cl:list 'trajectory_section
    (cl:cons ':points (points msg))
    (cl:cons ':end_point (end_point msg))
    (cl:cons ':start_point (start_point msg))
    (cl:cons ':z_height (z_height msg))
))

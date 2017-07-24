; Auto-generated. Do not edit!


(cl:in-package phd-msg)


;//! \htmlinclude trajectory_msg.msg.html

(cl:defclass <trajectory_msg> (roslisp-msg-protocol:ros-message)
  ((points
    :reader points
    :initarg :points
    :type (cl:vector phd-msg:trajectory_point)
   :initform (cl:make-array 0 :element-type 'phd-msg:trajectory_point :initial-element (cl:make-instance 'phd-msg:trajectory_point)))
   (start_pt
    :reader start_pt
    :initarg :start_pt
    :type phd-msg:trajectory_point
    :initform (cl:make-instance 'phd-msg:trajectory_point)))
)

(cl:defclass trajectory_msg (<trajectory_msg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <trajectory_msg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'trajectory_msg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name phd-msg:<trajectory_msg> is deprecated: use phd-msg:trajectory_msg instead.")))

(cl:ensure-generic-function 'points-val :lambda-list '(m))
(cl:defmethod points-val ((m <trajectory_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader phd-msg:points-val is deprecated.  Use phd-msg:points instead.")
  (points m))

(cl:ensure-generic-function 'start_pt-val :lambda-list '(m))
(cl:defmethod start_pt-val ((m <trajectory_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader phd-msg:start_pt-val is deprecated.  Use phd-msg:start_pt instead.")
  (start_pt m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <trajectory_msg>) ostream)
  "Serializes a message object of type '<trajectory_msg>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'points))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'points))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'start_pt) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <trajectory_msg>) istream)
  "Deserializes a message object of type '<trajectory_msg>"
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
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'start_pt) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<trajectory_msg>)))
  "Returns string type for a message object of type '<trajectory_msg>"
  "phd/trajectory_msg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'trajectory_msg)))
  "Returns string type for a message object of type 'trajectory_msg"
  "phd/trajectory_msg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<trajectory_msg>)))
  "Returns md5sum for a message object of type '<trajectory_msg>"
  "c33e31d3a8a75a28c1738126c9223a5c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'trajectory_msg)))
  "Returns md5sum for a message object of type 'trajectory_msg"
  "c33e31d3a8a75a28c1738126c9223a5c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<trajectory_msg>)))
  "Returns full string definition for message of type '<trajectory_msg>"
  (cl:format cl:nil "trajectory_point[] points~%trajectory_point start_pt~%~%================================================================================~%MSG: phd/trajectory_point~%float32 x~%float32 y~%float32 z~%float32 nx~%float32 ny~%float32 nz~%float32 d~%float32 d_abs~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'trajectory_msg)))
  "Returns full string definition for message of type 'trajectory_msg"
  (cl:format cl:nil "trajectory_point[] points~%trajectory_point start_pt~%~%================================================================================~%MSG: phd/trajectory_point~%float32 x~%float32 y~%float32 z~%float32 nx~%float32 ny~%float32 nz~%float32 d~%float32 d_abs~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <trajectory_msg>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'points) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'start_pt))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <trajectory_msg>))
  "Converts a ROS message object to a list"
  (cl:list 'trajectory_msg
    (cl:cons ':points (points msg))
    (cl:cons ':start_pt (start_pt msg))
))

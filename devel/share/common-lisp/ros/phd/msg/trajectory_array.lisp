; Auto-generated. Do not edit!


(cl:in-package phd-msg)


;//! \htmlinclude trajectory_array.msg.html

(cl:defclass <trajectory_array> (roslisp-msg-protocol:ros-message)
  ((sections
    :reader sections
    :initarg :sections
    :type (cl:vector phd-msg:trajectory_msg)
   :initform (cl:make-array 0 :element-type 'phd-msg:trajectory_msg :initial-element (cl:make-instance 'phd-msg:trajectory_msg))))
)

(cl:defclass trajectory_array (<trajectory_array>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <trajectory_array>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'trajectory_array)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name phd-msg:<trajectory_array> is deprecated: use phd-msg:trajectory_array instead.")))

(cl:ensure-generic-function 'sections-val :lambda-list '(m))
(cl:defmethod sections-val ((m <trajectory_array>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader phd-msg:sections-val is deprecated.  Use phd-msg:sections instead.")
  (sections m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <trajectory_array>) ostream)
  "Serializes a message object of type '<trajectory_array>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'sections))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'sections))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <trajectory_array>) istream)
  "Deserializes a message object of type '<trajectory_array>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'sections) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'sections)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'phd-msg:trajectory_msg))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<trajectory_array>)))
  "Returns string type for a message object of type '<trajectory_array>"
  "phd/trajectory_array")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'trajectory_array)))
  "Returns string type for a message object of type 'trajectory_array"
  "phd/trajectory_array")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<trajectory_array>)))
  "Returns md5sum for a message object of type '<trajectory_array>"
  "a2dbeff104317944dceb55b36edff5f0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'trajectory_array)))
  "Returns md5sum for a message object of type 'trajectory_array"
  "a2dbeff104317944dceb55b36edff5f0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<trajectory_array>)))
  "Returns full string definition for message of type '<trajectory_array>"
  (cl:format cl:nil "trajectory_msg[] sections~%~%~%================================================================================~%MSG: phd/trajectory_msg~%trajectory_point[] points~%trajectory_point start_pt~%~%================================================================================~%MSG: phd/trajectory_point~%float32 x~%float32 y~%float32 z~%float32 nx~%float32 ny~%float32 nz~%float32 d~%float32 d_abs~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'trajectory_array)))
  "Returns full string definition for message of type 'trajectory_array"
  (cl:format cl:nil "trajectory_msg[] sections~%~%~%================================================================================~%MSG: phd/trajectory_msg~%trajectory_point[] points~%trajectory_point start_pt~%~%================================================================================~%MSG: phd/trajectory_point~%float32 x~%float32 y~%float32 z~%float32 nx~%float32 ny~%float32 nz~%float32 d~%float32 d_abs~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <trajectory_array>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'sections) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <trajectory_array>))
  "Converts a ROS message object to a list"
  (cl:list 'trajectory_array
    (cl:cons ':sections (sections msg))
))

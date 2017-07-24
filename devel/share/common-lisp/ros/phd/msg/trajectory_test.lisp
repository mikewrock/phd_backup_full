; Auto-generated. Do not edit!


(cl:in-package phd-msg)


;//! \htmlinclude trajectory_test.msg.html

(cl:defclass <trajectory_test> (roslisp-msg-protocol:ros-message)
  ((x
    :reader x
    :initarg :x
    :type cl:float
    :initform 0.0))
)

(cl:defclass trajectory_test (<trajectory_test>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <trajectory_test>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'trajectory_test)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name phd-msg:<trajectory_test> is deprecated: use phd-msg:trajectory_test instead.")))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <trajectory_test>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader phd-msg:x-val is deprecated.  Use phd-msg:x instead.")
  (x m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <trajectory_test>) ostream)
  "Serializes a message object of type '<trajectory_test>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <trajectory_test>) istream)
  "Deserializes a message object of type '<trajectory_test>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'x) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<trajectory_test>)))
  "Returns string type for a message object of type '<trajectory_test>"
  "phd/trajectory_test")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'trajectory_test)))
  "Returns string type for a message object of type 'trajectory_test"
  "phd/trajectory_test")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<trajectory_test>)))
  "Returns md5sum for a message object of type '<trajectory_test>"
  "abd5d1e9c3ac157a0df3ba27b65d3384")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'trajectory_test)))
  "Returns md5sum for a message object of type 'trajectory_test"
  "abd5d1e9c3ac157a0df3ba27b65d3384")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<trajectory_test>)))
  "Returns full string definition for message of type '<trajectory_test>"
  (cl:format cl:nil "float32 x~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'trajectory_test)))
  "Returns full string definition for message of type 'trajectory_test"
  (cl:format cl:nil "float32 x~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <trajectory_test>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <trajectory_test>))
  "Converts a ROS message object to a list"
  (cl:list 'trajectory_test
    (cl:cons ':x (x msg))
))

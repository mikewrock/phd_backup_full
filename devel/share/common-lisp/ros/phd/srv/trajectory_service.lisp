; Auto-generated. Do not edit!


(cl:in-package phd-srv)


;//! \htmlinclude trajectory_service-request.msg.html

(cl:defclass <trajectory_service-request> (roslisp-msg-protocol:ros-message)
  ((P1x
    :reader P1x
    :initarg :P1x
    :type cl:float
    :initform 0.0)
   (P1y
    :reader P1y
    :initarg :P1y
    :type cl:float
    :initform 0.0)
   (P1z
    :reader P1z
    :initarg :P1z
    :type cl:float
    :initform 0.0)
   (P2x
    :reader P2x
    :initarg :P2x
    :type cl:float
    :initform 0.0)
   (P2y
    :reader P2y
    :initarg :P2y
    :type cl:float
    :initform 0.0)
   (P2z
    :reader P2z
    :initarg :P2z
    :type cl:float
    :initform 0.0)
   (P3x
    :reader P3x
    :initarg :P3x
    :type cl:float
    :initform 0.0)
   (P3y
    :reader P3y
    :initarg :P3y
    :type cl:float
    :initform 0.0)
   (P3z
    :reader P3z
    :initarg :P3z
    :type cl:float
    :initform 0.0)
   (cloud_in
    :reader cloud_in
    :initarg :cloud_in
    :type sensor_msgs-msg:PointCloud2
    :initform (cl:make-instance 'sensor_msgs-msg:PointCloud2))
   (cloud_surface
    :reader cloud_surface
    :initarg :cloud_surface
    :type sensor_msgs-msg:PointCloud2
    :initform (cl:make-instance 'sensor_msgs-msg:PointCloud2)))
)

(cl:defclass trajectory_service-request (<trajectory_service-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <trajectory_service-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'trajectory_service-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name phd-srv:<trajectory_service-request> is deprecated: use phd-srv:trajectory_service-request instead.")))

(cl:ensure-generic-function 'P1x-val :lambda-list '(m))
(cl:defmethod P1x-val ((m <trajectory_service-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader phd-srv:P1x-val is deprecated.  Use phd-srv:P1x instead.")
  (P1x m))

(cl:ensure-generic-function 'P1y-val :lambda-list '(m))
(cl:defmethod P1y-val ((m <trajectory_service-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader phd-srv:P1y-val is deprecated.  Use phd-srv:P1y instead.")
  (P1y m))

(cl:ensure-generic-function 'P1z-val :lambda-list '(m))
(cl:defmethod P1z-val ((m <trajectory_service-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader phd-srv:P1z-val is deprecated.  Use phd-srv:P1z instead.")
  (P1z m))

(cl:ensure-generic-function 'P2x-val :lambda-list '(m))
(cl:defmethod P2x-val ((m <trajectory_service-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader phd-srv:P2x-val is deprecated.  Use phd-srv:P2x instead.")
  (P2x m))

(cl:ensure-generic-function 'P2y-val :lambda-list '(m))
(cl:defmethod P2y-val ((m <trajectory_service-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader phd-srv:P2y-val is deprecated.  Use phd-srv:P2y instead.")
  (P2y m))

(cl:ensure-generic-function 'P2z-val :lambda-list '(m))
(cl:defmethod P2z-val ((m <trajectory_service-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader phd-srv:P2z-val is deprecated.  Use phd-srv:P2z instead.")
  (P2z m))

(cl:ensure-generic-function 'P3x-val :lambda-list '(m))
(cl:defmethod P3x-val ((m <trajectory_service-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader phd-srv:P3x-val is deprecated.  Use phd-srv:P3x instead.")
  (P3x m))

(cl:ensure-generic-function 'P3y-val :lambda-list '(m))
(cl:defmethod P3y-val ((m <trajectory_service-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader phd-srv:P3y-val is deprecated.  Use phd-srv:P3y instead.")
  (P3y m))

(cl:ensure-generic-function 'P3z-val :lambda-list '(m))
(cl:defmethod P3z-val ((m <trajectory_service-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader phd-srv:P3z-val is deprecated.  Use phd-srv:P3z instead.")
  (P3z m))

(cl:ensure-generic-function 'cloud_in-val :lambda-list '(m))
(cl:defmethod cloud_in-val ((m <trajectory_service-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader phd-srv:cloud_in-val is deprecated.  Use phd-srv:cloud_in instead.")
  (cloud_in m))

(cl:ensure-generic-function 'cloud_surface-val :lambda-list '(m))
(cl:defmethod cloud_surface-val ((m <trajectory_service-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader phd-srv:cloud_surface-val is deprecated.  Use phd-srv:cloud_surface instead.")
  (cloud_surface m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <trajectory_service-request>) ostream)
  "Serializes a message object of type '<trajectory_service-request>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'P1x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'P1y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'P1z))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'P2x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'P2y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'P2z))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'P3x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'P3y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'P3z))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'cloud_in) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'cloud_surface) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <trajectory_service-request>) istream)
  "Deserializes a message object of type '<trajectory_service-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'P1x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'P1y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'P1z) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'P2x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'P2y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'P2z) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'P3x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'P3y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'P3z) (roslisp-utils:decode-single-float-bits bits)))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'cloud_in) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'cloud_surface) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<trajectory_service-request>)))
  "Returns string type for a service object of type '<trajectory_service-request>"
  "phd/trajectory_serviceRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'trajectory_service-request)))
  "Returns string type for a service object of type 'trajectory_service-request"
  "phd/trajectory_serviceRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<trajectory_service-request>)))
  "Returns md5sum for a message object of type '<trajectory_service-request>"
  "406cb769178a6d411779375e331fbd67")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'trajectory_service-request)))
  "Returns md5sum for a message object of type 'trajectory_service-request"
  "406cb769178a6d411779375e331fbd67")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<trajectory_service-request>)))
  "Returns full string definition for message of type '<trajectory_service-request>"
  (cl:format cl:nil "float32 P1x~%float32 P1y~%float32 P1z~%float32 P2x~%float32 P2y~%float32 P2z~%float32 P3x~%float32 P3y~%float32 P3z~%sensor_msgs/PointCloud2 cloud_in~%sensor_msgs/PointCloud2 cloud_surface~%~%================================================================================~%MSG: sensor_msgs/PointCloud2~%# This message holds a collection of N-dimensional points, which may~%# contain additional information such as normals, intensity, etc. The~%# point data is stored as a binary blob, its layout described by the~%# contents of the \"fields\" array.~%~%# The point cloud data may be organized 2d (image-like) or 1d~%# (unordered). Point clouds organized as 2d images may be produced by~%# camera depth sensors such as stereo or time-of-flight.~%~%# Time of sensor data acquisition, and the coordinate frame ID (for 3d~%# points).~%Header header~%~%# 2D structure of the point cloud. If the cloud is unordered, height is~%# 1 and width is the length of the point cloud.~%uint32 height~%uint32 width~%~%# Describes the channels and their layout in the binary data blob.~%PointField[] fields~%~%bool    is_bigendian # Is this data bigendian?~%uint32  point_step   # Length of a point in bytes~%uint32  row_step     # Length of a row in bytes~%uint8[] data         # Actual point data, size is (row_step*height)~%~%bool is_dense        # True if there are no invalid points~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: sensor_msgs/PointField~%# This message holds the description of one point entry in the~%# PointCloud2 message format.~%uint8 INT8    = 1~%uint8 UINT8   = 2~%uint8 INT16   = 3~%uint8 UINT16  = 4~%uint8 INT32   = 5~%uint8 UINT32  = 6~%uint8 FLOAT32 = 7~%uint8 FLOAT64 = 8~%~%string name      # Name of field~%uint32 offset    # Offset from start of point struct~%uint8  datatype  # Datatype enumeration, see above~%uint32 count     # How many elements in the field~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'trajectory_service-request)))
  "Returns full string definition for message of type 'trajectory_service-request"
  (cl:format cl:nil "float32 P1x~%float32 P1y~%float32 P1z~%float32 P2x~%float32 P2y~%float32 P2z~%float32 P3x~%float32 P3y~%float32 P3z~%sensor_msgs/PointCloud2 cloud_in~%sensor_msgs/PointCloud2 cloud_surface~%~%================================================================================~%MSG: sensor_msgs/PointCloud2~%# This message holds a collection of N-dimensional points, which may~%# contain additional information such as normals, intensity, etc. The~%# point data is stored as a binary blob, its layout described by the~%# contents of the \"fields\" array.~%~%# The point cloud data may be organized 2d (image-like) or 1d~%# (unordered). Point clouds organized as 2d images may be produced by~%# camera depth sensors such as stereo or time-of-flight.~%~%# Time of sensor data acquisition, and the coordinate frame ID (for 3d~%# points).~%Header header~%~%# 2D structure of the point cloud. If the cloud is unordered, height is~%# 1 and width is the length of the point cloud.~%uint32 height~%uint32 width~%~%# Describes the channels and their layout in the binary data blob.~%PointField[] fields~%~%bool    is_bigendian # Is this data bigendian?~%uint32  point_step   # Length of a point in bytes~%uint32  row_step     # Length of a row in bytes~%uint8[] data         # Actual point data, size is (row_step*height)~%~%bool is_dense        # True if there are no invalid points~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: sensor_msgs/PointField~%# This message holds the description of one point entry in the~%# PointCloud2 message format.~%uint8 INT8    = 1~%uint8 UINT8   = 2~%uint8 INT16   = 3~%uint8 UINT16  = 4~%uint8 INT32   = 5~%uint8 UINT32  = 6~%uint8 FLOAT32 = 7~%uint8 FLOAT64 = 8~%~%string name      # Name of field~%uint32 offset    # Offset from start of point struct~%uint8  datatype  # Datatype enumeration, see above~%uint32 count     # How many elements in the field~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <trajectory_service-request>))
  (cl:+ 0
     4
     4
     4
     4
     4
     4
     4
     4
     4
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'cloud_in))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'cloud_surface))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <trajectory_service-request>))
  "Converts a ROS message object to a list"
  (cl:list 'trajectory_service-request
    (cl:cons ':P1x (P1x msg))
    (cl:cons ':P1y (P1y msg))
    (cl:cons ':P1z (P1z msg))
    (cl:cons ':P2x (P2x msg))
    (cl:cons ':P2y (P2y msg))
    (cl:cons ':P2z (P2z msg))
    (cl:cons ':P3x (P3x msg))
    (cl:cons ':P3y (P3y msg))
    (cl:cons ':P3z (P3z msg))
    (cl:cons ':cloud_in (cloud_in msg))
    (cl:cons ':cloud_surface (cloud_surface msg))
))
;//! \htmlinclude trajectory_service-response.msg.html

(cl:defclass <trajectory_service-response> (roslisp-msg-protocol:ros-message)
  ((trajectory
    :reader trajectory
    :initarg :trajectory
    :type phd-msg:trajectory_array
    :initform (cl:make-instance 'phd-msg:trajectory_array)))
)

(cl:defclass trajectory_service-response (<trajectory_service-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <trajectory_service-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'trajectory_service-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name phd-srv:<trajectory_service-response> is deprecated: use phd-srv:trajectory_service-response instead.")))

(cl:ensure-generic-function 'trajectory-val :lambda-list '(m))
(cl:defmethod trajectory-val ((m <trajectory_service-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader phd-srv:trajectory-val is deprecated.  Use phd-srv:trajectory instead.")
  (trajectory m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <trajectory_service-response>) ostream)
  "Serializes a message object of type '<trajectory_service-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'trajectory) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <trajectory_service-response>) istream)
  "Deserializes a message object of type '<trajectory_service-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'trajectory) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<trajectory_service-response>)))
  "Returns string type for a service object of type '<trajectory_service-response>"
  "phd/trajectory_serviceResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'trajectory_service-response)))
  "Returns string type for a service object of type 'trajectory_service-response"
  "phd/trajectory_serviceResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<trajectory_service-response>)))
  "Returns md5sum for a message object of type '<trajectory_service-response>"
  "406cb769178a6d411779375e331fbd67")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'trajectory_service-response)))
  "Returns md5sum for a message object of type 'trajectory_service-response"
  "406cb769178a6d411779375e331fbd67")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<trajectory_service-response>)))
  "Returns full string definition for message of type '<trajectory_service-response>"
  (cl:format cl:nil "trajectory_array trajectory~%~%~%================================================================================~%MSG: phd/trajectory_array~%trajectory_msg[] sections~%~%~%================================================================================~%MSG: phd/trajectory_msg~%trajectory_point[] points~%trajectory_point start_pt~%~%================================================================================~%MSG: phd/trajectory_point~%float32 x~%float32 y~%float32 z~%float32 nx~%float32 ny~%float32 nz~%float32 d~%float32 d_abs~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'trajectory_service-response)))
  "Returns full string definition for message of type 'trajectory_service-response"
  (cl:format cl:nil "trajectory_array trajectory~%~%~%================================================================================~%MSG: phd/trajectory_array~%trajectory_msg[] sections~%~%~%================================================================================~%MSG: phd/trajectory_msg~%trajectory_point[] points~%trajectory_point start_pt~%~%================================================================================~%MSG: phd/trajectory_point~%float32 x~%float32 y~%float32 z~%float32 nx~%float32 ny~%float32 nz~%float32 d~%float32 d_abs~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <trajectory_service-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'trajectory))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <trajectory_service-response>))
  "Converts a ROS message object to a list"
  (cl:list 'trajectory_service-response
    (cl:cons ':trajectory (trajectory msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'trajectory_service)))
  'trajectory_service-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'trajectory_service)))
  'trajectory_service-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'trajectory_service)))
  "Returns string type for a service object of type '<trajectory_service>"
  "phd/trajectory_service")
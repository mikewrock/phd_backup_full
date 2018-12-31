; Auto-generated. Do not edit!


(cl:in-package phd-srv)


;//! \htmlinclude accuracy_service-request.msg.html

(cl:defclass <accuracy_service-request> (roslisp-msg-protocol:ros-message)
  ((cloud_1
    :reader cloud_1
    :initarg :cloud_1
    :type sensor_msgs-msg:PointCloud2
    :initform (cl:make-instance 'sensor_msgs-msg:PointCloud2))
   (cloud_2
    :reader cloud_2
    :initarg :cloud_2
    :type sensor_msgs-msg:PointCloud2
    :initform (cl:make-instance 'sensor_msgs-msg:PointCloud2))
   (cloud_datum
    :reader cloud_datum
    :initarg :cloud_datum
    :type sensor_msgs-msg:PointCloud2
    :initform (cl:make-instance 'sensor_msgs-msg:PointCloud2)))
)

(cl:defclass accuracy_service-request (<accuracy_service-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <accuracy_service-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'accuracy_service-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name phd-srv:<accuracy_service-request> is deprecated: use phd-srv:accuracy_service-request instead.")))

(cl:ensure-generic-function 'cloud_1-val :lambda-list '(m))
(cl:defmethod cloud_1-val ((m <accuracy_service-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader phd-srv:cloud_1-val is deprecated.  Use phd-srv:cloud_1 instead.")
  (cloud_1 m))

(cl:ensure-generic-function 'cloud_2-val :lambda-list '(m))
(cl:defmethod cloud_2-val ((m <accuracy_service-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader phd-srv:cloud_2-val is deprecated.  Use phd-srv:cloud_2 instead.")
  (cloud_2 m))

(cl:ensure-generic-function 'cloud_datum-val :lambda-list '(m))
(cl:defmethod cloud_datum-val ((m <accuracy_service-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader phd-srv:cloud_datum-val is deprecated.  Use phd-srv:cloud_datum instead.")
  (cloud_datum m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <accuracy_service-request>) ostream)
  "Serializes a message object of type '<accuracy_service-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'cloud_1) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'cloud_2) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'cloud_datum) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <accuracy_service-request>) istream)
  "Deserializes a message object of type '<accuracy_service-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'cloud_1) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'cloud_2) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'cloud_datum) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<accuracy_service-request>)))
  "Returns string type for a service object of type '<accuracy_service-request>"
  "phd/accuracy_serviceRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'accuracy_service-request)))
  "Returns string type for a service object of type 'accuracy_service-request"
  "phd/accuracy_serviceRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<accuracy_service-request>)))
  "Returns md5sum for a message object of type '<accuracy_service-request>"
  "d6902a0a52e4f694a57d510cce74d5d2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'accuracy_service-request)))
  "Returns md5sum for a message object of type 'accuracy_service-request"
  "d6902a0a52e4f694a57d510cce74d5d2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<accuracy_service-request>)))
  "Returns full string definition for message of type '<accuracy_service-request>"
  (cl:format cl:nil "sensor_msgs/PointCloud2 cloud_1~%sensor_msgs/PointCloud2 cloud_2~%sensor_msgs/PointCloud2 cloud_datum~%~%================================================================================~%MSG: sensor_msgs/PointCloud2~%# This message holds a collection of N-dimensional points, which may~%# contain additional information such as normals, intensity, etc. The~%# point data is stored as a binary blob, its layout described by the~%# contents of the \"fields\" array.~%~%# The point cloud data may be organized 2d (image-like) or 1d~%# (unordered). Point clouds organized as 2d images may be produced by~%# camera depth sensors such as stereo or time-of-flight.~%~%# Time of sensor data acquisition, and the coordinate frame ID (for 3d~%# points).~%Header header~%~%# 2D structure of the point cloud. If the cloud is unordered, height is~%# 1 and width is the length of the point cloud.~%uint32 height~%uint32 width~%~%# Describes the channels and their layout in the binary data blob.~%PointField[] fields~%~%bool    is_bigendian # Is this data bigendian?~%uint32  point_step   # Length of a point in bytes~%uint32  row_step     # Length of a row in bytes~%uint8[] data         # Actual point data, size is (row_step*height)~%~%bool is_dense        # True if there are no invalid points~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: sensor_msgs/PointField~%# This message holds the description of one point entry in the~%# PointCloud2 message format.~%uint8 INT8    = 1~%uint8 UINT8   = 2~%uint8 INT16   = 3~%uint8 UINT16  = 4~%uint8 INT32   = 5~%uint8 UINT32  = 6~%uint8 FLOAT32 = 7~%uint8 FLOAT64 = 8~%~%string name      # Name of field~%uint32 offset    # Offset from start of point struct~%uint8  datatype  # Datatype enumeration, see above~%uint32 count     # How many elements in the field~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'accuracy_service-request)))
  "Returns full string definition for message of type 'accuracy_service-request"
  (cl:format cl:nil "sensor_msgs/PointCloud2 cloud_1~%sensor_msgs/PointCloud2 cloud_2~%sensor_msgs/PointCloud2 cloud_datum~%~%================================================================================~%MSG: sensor_msgs/PointCloud2~%# This message holds a collection of N-dimensional points, which may~%# contain additional information such as normals, intensity, etc. The~%# point data is stored as a binary blob, its layout described by the~%# contents of the \"fields\" array.~%~%# The point cloud data may be organized 2d (image-like) or 1d~%# (unordered). Point clouds organized as 2d images may be produced by~%# camera depth sensors such as stereo or time-of-flight.~%~%# Time of sensor data acquisition, and the coordinate frame ID (for 3d~%# points).~%Header header~%~%# 2D structure of the point cloud. If the cloud is unordered, height is~%# 1 and width is the length of the point cloud.~%uint32 height~%uint32 width~%~%# Describes the channels and their layout in the binary data blob.~%PointField[] fields~%~%bool    is_bigendian # Is this data bigendian?~%uint32  point_step   # Length of a point in bytes~%uint32  row_step     # Length of a row in bytes~%uint8[] data         # Actual point data, size is (row_step*height)~%~%bool is_dense        # True if there are no invalid points~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: sensor_msgs/PointField~%# This message holds the description of one point entry in the~%# PointCloud2 message format.~%uint8 INT8    = 1~%uint8 UINT8   = 2~%uint8 INT16   = 3~%uint8 UINT16  = 4~%uint8 INT32   = 5~%uint8 UINT32  = 6~%uint8 FLOAT32 = 7~%uint8 FLOAT64 = 8~%~%string name      # Name of field~%uint32 offset    # Offset from start of point struct~%uint8  datatype  # Datatype enumeration, see above~%uint32 count     # How many elements in the field~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <accuracy_service-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'cloud_1))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'cloud_2))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'cloud_datum))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <accuracy_service-request>))
  "Converts a ROS message object to a list"
  (cl:list 'accuracy_service-request
    (cl:cons ':cloud_1 (cloud_1 msg))
    (cl:cons ':cloud_2 (cloud_2 msg))
    (cl:cons ':cloud_datum (cloud_datum msg))
))
;//! \htmlinclude accuracy_service-response.msg.html

(cl:defclass <accuracy_service-response> (roslisp-msg-protocol:ros-message)
  ((cloud_out
    :reader cloud_out
    :initarg :cloud_out
    :type sensor_msgs-msg:PointCloud2
    :initform (cl:make-instance 'sensor_msgs-msg:PointCloud2)))
)

(cl:defclass accuracy_service-response (<accuracy_service-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <accuracy_service-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'accuracy_service-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name phd-srv:<accuracy_service-response> is deprecated: use phd-srv:accuracy_service-response instead.")))

(cl:ensure-generic-function 'cloud_out-val :lambda-list '(m))
(cl:defmethod cloud_out-val ((m <accuracy_service-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader phd-srv:cloud_out-val is deprecated.  Use phd-srv:cloud_out instead.")
  (cloud_out m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <accuracy_service-response>) ostream)
  "Serializes a message object of type '<accuracy_service-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'cloud_out) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <accuracy_service-response>) istream)
  "Deserializes a message object of type '<accuracy_service-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'cloud_out) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<accuracy_service-response>)))
  "Returns string type for a service object of type '<accuracy_service-response>"
  "phd/accuracy_serviceResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'accuracy_service-response)))
  "Returns string type for a service object of type 'accuracy_service-response"
  "phd/accuracy_serviceResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<accuracy_service-response>)))
  "Returns md5sum for a message object of type '<accuracy_service-response>"
  "d6902a0a52e4f694a57d510cce74d5d2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'accuracy_service-response)))
  "Returns md5sum for a message object of type 'accuracy_service-response"
  "d6902a0a52e4f694a57d510cce74d5d2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<accuracy_service-response>)))
  "Returns full string definition for message of type '<accuracy_service-response>"
  (cl:format cl:nil "sensor_msgs/PointCloud2 cloud_out~%~%~%================================================================================~%MSG: sensor_msgs/PointCloud2~%# This message holds a collection of N-dimensional points, which may~%# contain additional information such as normals, intensity, etc. The~%# point data is stored as a binary blob, its layout described by the~%# contents of the \"fields\" array.~%~%# The point cloud data may be organized 2d (image-like) or 1d~%# (unordered). Point clouds organized as 2d images may be produced by~%# camera depth sensors such as stereo or time-of-flight.~%~%# Time of sensor data acquisition, and the coordinate frame ID (for 3d~%# points).~%Header header~%~%# 2D structure of the point cloud. If the cloud is unordered, height is~%# 1 and width is the length of the point cloud.~%uint32 height~%uint32 width~%~%# Describes the channels and their layout in the binary data blob.~%PointField[] fields~%~%bool    is_bigendian # Is this data bigendian?~%uint32  point_step   # Length of a point in bytes~%uint32  row_step     # Length of a row in bytes~%uint8[] data         # Actual point data, size is (row_step*height)~%~%bool is_dense        # True if there are no invalid points~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: sensor_msgs/PointField~%# This message holds the description of one point entry in the~%# PointCloud2 message format.~%uint8 INT8    = 1~%uint8 UINT8   = 2~%uint8 INT16   = 3~%uint8 UINT16  = 4~%uint8 INT32   = 5~%uint8 UINT32  = 6~%uint8 FLOAT32 = 7~%uint8 FLOAT64 = 8~%~%string name      # Name of field~%uint32 offset    # Offset from start of point struct~%uint8  datatype  # Datatype enumeration, see above~%uint32 count     # How many elements in the field~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'accuracy_service-response)))
  "Returns full string definition for message of type 'accuracy_service-response"
  (cl:format cl:nil "sensor_msgs/PointCloud2 cloud_out~%~%~%================================================================================~%MSG: sensor_msgs/PointCloud2~%# This message holds a collection of N-dimensional points, which may~%# contain additional information such as normals, intensity, etc. The~%# point data is stored as a binary blob, its layout described by the~%# contents of the \"fields\" array.~%~%# The point cloud data may be organized 2d (image-like) or 1d~%# (unordered). Point clouds organized as 2d images may be produced by~%# camera depth sensors such as stereo or time-of-flight.~%~%# Time of sensor data acquisition, and the coordinate frame ID (for 3d~%# points).~%Header header~%~%# 2D structure of the point cloud. If the cloud is unordered, height is~%# 1 and width is the length of the point cloud.~%uint32 height~%uint32 width~%~%# Describes the channels and their layout in the binary data blob.~%PointField[] fields~%~%bool    is_bigendian # Is this data bigendian?~%uint32  point_step   # Length of a point in bytes~%uint32  row_step     # Length of a row in bytes~%uint8[] data         # Actual point data, size is (row_step*height)~%~%bool is_dense        # True if there are no invalid points~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: sensor_msgs/PointField~%# This message holds the description of one point entry in the~%# PointCloud2 message format.~%uint8 INT8    = 1~%uint8 UINT8   = 2~%uint8 INT16   = 3~%uint8 UINT16  = 4~%uint8 INT32   = 5~%uint8 UINT32  = 6~%uint8 FLOAT32 = 7~%uint8 FLOAT64 = 8~%~%string name      # Name of field~%uint32 offset    # Offset from start of point struct~%uint8  datatype  # Datatype enumeration, see above~%uint32 count     # How many elements in the field~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <accuracy_service-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'cloud_out))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <accuracy_service-response>))
  "Converts a ROS message object to a list"
  (cl:list 'accuracy_service-response
    (cl:cons ':cloud_out (cloud_out msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'accuracy_service)))
  'accuracy_service-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'accuracy_service)))
  'accuracy_service-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'accuracy_service)))
  "Returns string type for a service object of type '<accuracy_service>"
  "phd/accuracy_service")
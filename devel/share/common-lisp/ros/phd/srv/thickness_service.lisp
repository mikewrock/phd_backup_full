; Auto-generated. Do not edit!


(cl:in-package phd-srv)


;//! \htmlinclude thickness_service-request.msg.html

(cl:defclass <thickness_service-request> (roslisp-msg-protocol:ros-message)
  ((cloud_1
    :reader cloud_1
    :initarg :cloud_1
    :type sensor_msgs-msg:PointCloud2
    :initform (cl:make-instance 'sensor_msgs-msg:PointCloud2))
   (cloud_2
    :reader cloud_2
    :initarg :cloud_2
    :type sensor_msgs-msg:PointCloud2
    :initform (cl:make-instance 'sensor_msgs-msg:PointCloud2)))
)

(cl:defclass thickness_service-request (<thickness_service-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <thickness_service-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'thickness_service-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name phd-srv:<thickness_service-request> is deprecated: use phd-srv:thickness_service-request instead.")))

(cl:ensure-generic-function 'cloud_1-val :lambda-list '(m))
(cl:defmethod cloud_1-val ((m <thickness_service-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader phd-srv:cloud_1-val is deprecated.  Use phd-srv:cloud_1 instead.")
  (cloud_1 m))

(cl:ensure-generic-function 'cloud_2-val :lambda-list '(m))
(cl:defmethod cloud_2-val ((m <thickness_service-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader phd-srv:cloud_2-val is deprecated.  Use phd-srv:cloud_2 instead.")
  (cloud_2 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <thickness_service-request>) ostream)
  "Serializes a message object of type '<thickness_service-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'cloud_1) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'cloud_2) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <thickness_service-request>) istream)
  "Deserializes a message object of type '<thickness_service-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'cloud_1) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'cloud_2) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<thickness_service-request>)))
  "Returns string type for a service object of type '<thickness_service-request>"
  "phd/thickness_serviceRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'thickness_service-request)))
  "Returns string type for a service object of type 'thickness_service-request"
  "phd/thickness_serviceRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<thickness_service-request>)))
  "Returns md5sum for a message object of type '<thickness_service-request>"
  "0fa11251ff61f1cb78f9a7e04b6af3f2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'thickness_service-request)))
  "Returns md5sum for a message object of type 'thickness_service-request"
  "0fa11251ff61f1cb78f9a7e04b6af3f2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<thickness_service-request>)))
  "Returns full string definition for message of type '<thickness_service-request>"
  (cl:format cl:nil "sensor_msgs/PointCloud2 cloud_1~%sensor_msgs/PointCloud2 cloud_2~%~%================================================================================~%MSG: sensor_msgs/PointCloud2~%# This message holds a collection of N-dimensional points, which may~%# contain additional information such as normals, intensity, etc. The~%# point data is stored as a binary blob, its layout described by the~%# contents of the \"fields\" array.~%~%# The point cloud data may be organized 2d (image-like) or 1d~%# (unordered). Point clouds organized as 2d images may be produced by~%# camera depth sensors such as stereo or time-of-flight.~%~%# Time of sensor data acquisition, and the coordinate frame ID (for 3d~%# points).~%Header header~%~%# 2D structure of the point cloud. If the cloud is unordered, height is~%# 1 and width is the length of the point cloud.~%uint32 height~%uint32 width~%~%# Describes the channels and their layout in the binary data blob.~%PointField[] fields~%~%bool    is_bigendian # Is this data bigendian?~%uint32  point_step   # Length of a point in bytes~%uint32  row_step     # Length of a row in bytes~%uint8[] data         # Actual point data, size is (row_step*height)~%~%bool is_dense        # True if there are no invalid points~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: sensor_msgs/PointField~%# This message holds the description of one point entry in the~%# PointCloud2 message format.~%uint8 INT8    = 1~%uint8 UINT8   = 2~%uint8 INT16   = 3~%uint8 UINT16  = 4~%uint8 INT32   = 5~%uint8 UINT32  = 6~%uint8 FLOAT32 = 7~%uint8 FLOAT64 = 8~%~%string name      # Name of field~%uint32 offset    # Offset from start of point struct~%uint8  datatype  # Datatype enumeration, see above~%uint32 count     # How many elements in the field~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'thickness_service-request)))
  "Returns full string definition for message of type 'thickness_service-request"
  (cl:format cl:nil "sensor_msgs/PointCloud2 cloud_1~%sensor_msgs/PointCloud2 cloud_2~%~%================================================================================~%MSG: sensor_msgs/PointCloud2~%# This message holds a collection of N-dimensional points, which may~%# contain additional information such as normals, intensity, etc. The~%# point data is stored as a binary blob, its layout described by the~%# contents of the \"fields\" array.~%~%# The point cloud data may be organized 2d (image-like) or 1d~%# (unordered). Point clouds organized as 2d images may be produced by~%# camera depth sensors such as stereo or time-of-flight.~%~%# Time of sensor data acquisition, and the coordinate frame ID (for 3d~%# points).~%Header header~%~%# 2D structure of the point cloud. If the cloud is unordered, height is~%# 1 and width is the length of the point cloud.~%uint32 height~%uint32 width~%~%# Describes the channels and their layout in the binary data blob.~%PointField[] fields~%~%bool    is_bigendian # Is this data bigendian?~%uint32  point_step   # Length of a point in bytes~%uint32  row_step     # Length of a row in bytes~%uint8[] data         # Actual point data, size is (row_step*height)~%~%bool is_dense        # True if there are no invalid points~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: sensor_msgs/PointField~%# This message holds the description of one point entry in the~%# PointCloud2 message format.~%uint8 INT8    = 1~%uint8 UINT8   = 2~%uint8 INT16   = 3~%uint8 UINT16  = 4~%uint8 INT32   = 5~%uint8 UINT32  = 6~%uint8 FLOAT32 = 7~%uint8 FLOAT64 = 8~%~%string name      # Name of field~%uint32 offset    # Offset from start of point struct~%uint8  datatype  # Datatype enumeration, see above~%uint32 count     # How many elements in the field~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <thickness_service-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'cloud_1))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'cloud_2))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <thickness_service-request>))
  "Converts a ROS message object to a list"
  (cl:list 'thickness_service-request
    (cl:cons ':cloud_1 (cloud_1 msg))
    (cl:cons ':cloud_2 (cloud_2 msg))
))
;//! \htmlinclude thickness_service-response.msg.html

(cl:defclass <thickness_service-response> (roslisp-msg-protocol:ros-message)
  ((cloud_out
    :reader cloud_out
    :initarg :cloud_out
    :type sensor_msgs-msg:PointCloud2
    :initform (cl:make-instance 'sensor_msgs-msg:PointCloud2)))
)

(cl:defclass thickness_service-response (<thickness_service-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <thickness_service-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'thickness_service-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name phd-srv:<thickness_service-response> is deprecated: use phd-srv:thickness_service-response instead.")))

(cl:ensure-generic-function 'cloud_out-val :lambda-list '(m))
(cl:defmethod cloud_out-val ((m <thickness_service-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader phd-srv:cloud_out-val is deprecated.  Use phd-srv:cloud_out instead.")
  (cloud_out m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <thickness_service-response>) ostream)
  "Serializes a message object of type '<thickness_service-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'cloud_out) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <thickness_service-response>) istream)
  "Deserializes a message object of type '<thickness_service-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'cloud_out) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<thickness_service-response>)))
  "Returns string type for a service object of type '<thickness_service-response>"
  "phd/thickness_serviceResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'thickness_service-response)))
  "Returns string type for a service object of type 'thickness_service-response"
  "phd/thickness_serviceResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<thickness_service-response>)))
  "Returns md5sum for a message object of type '<thickness_service-response>"
  "0fa11251ff61f1cb78f9a7e04b6af3f2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'thickness_service-response)))
  "Returns md5sum for a message object of type 'thickness_service-response"
  "0fa11251ff61f1cb78f9a7e04b6af3f2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<thickness_service-response>)))
  "Returns full string definition for message of type '<thickness_service-response>"
  (cl:format cl:nil "sensor_msgs/PointCloud2 cloud_out~%~%~%================================================================================~%MSG: sensor_msgs/PointCloud2~%# This message holds a collection of N-dimensional points, which may~%# contain additional information such as normals, intensity, etc. The~%# point data is stored as a binary blob, its layout described by the~%# contents of the \"fields\" array.~%~%# The point cloud data may be organized 2d (image-like) or 1d~%# (unordered). Point clouds organized as 2d images may be produced by~%# camera depth sensors such as stereo or time-of-flight.~%~%# Time of sensor data acquisition, and the coordinate frame ID (for 3d~%# points).~%Header header~%~%# 2D structure of the point cloud. If the cloud is unordered, height is~%# 1 and width is the length of the point cloud.~%uint32 height~%uint32 width~%~%# Describes the channels and their layout in the binary data blob.~%PointField[] fields~%~%bool    is_bigendian # Is this data bigendian?~%uint32  point_step   # Length of a point in bytes~%uint32  row_step     # Length of a row in bytes~%uint8[] data         # Actual point data, size is (row_step*height)~%~%bool is_dense        # True if there are no invalid points~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: sensor_msgs/PointField~%# This message holds the description of one point entry in the~%# PointCloud2 message format.~%uint8 INT8    = 1~%uint8 UINT8   = 2~%uint8 INT16   = 3~%uint8 UINT16  = 4~%uint8 INT32   = 5~%uint8 UINT32  = 6~%uint8 FLOAT32 = 7~%uint8 FLOAT64 = 8~%~%string name      # Name of field~%uint32 offset    # Offset from start of point struct~%uint8  datatype  # Datatype enumeration, see above~%uint32 count     # How many elements in the field~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'thickness_service-response)))
  "Returns full string definition for message of type 'thickness_service-response"
  (cl:format cl:nil "sensor_msgs/PointCloud2 cloud_out~%~%~%================================================================================~%MSG: sensor_msgs/PointCloud2~%# This message holds a collection of N-dimensional points, which may~%# contain additional information such as normals, intensity, etc. The~%# point data is stored as a binary blob, its layout described by the~%# contents of the \"fields\" array.~%~%# The point cloud data may be organized 2d (image-like) or 1d~%# (unordered). Point clouds organized as 2d images may be produced by~%# camera depth sensors such as stereo or time-of-flight.~%~%# Time of sensor data acquisition, and the coordinate frame ID (for 3d~%# points).~%Header header~%~%# 2D structure of the point cloud. If the cloud is unordered, height is~%# 1 and width is the length of the point cloud.~%uint32 height~%uint32 width~%~%# Describes the channels and their layout in the binary data blob.~%PointField[] fields~%~%bool    is_bigendian # Is this data bigendian?~%uint32  point_step   # Length of a point in bytes~%uint32  row_step     # Length of a row in bytes~%uint8[] data         # Actual point data, size is (row_step*height)~%~%bool is_dense        # True if there are no invalid points~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: sensor_msgs/PointField~%# This message holds the description of one point entry in the~%# PointCloud2 message format.~%uint8 INT8    = 1~%uint8 UINT8   = 2~%uint8 INT16   = 3~%uint8 UINT16  = 4~%uint8 INT32   = 5~%uint8 UINT32  = 6~%uint8 FLOAT32 = 7~%uint8 FLOAT64 = 8~%~%string name      # Name of field~%uint32 offset    # Offset from start of point struct~%uint8  datatype  # Datatype enumeration, see above~%uint32 count     # How many elements in the field~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <thickness_service-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'cloud_out))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <thickness_service-response>))
  "Converts a ROS message object to a list"
  (cl:list 'thickness_service-response
    (cl:cons ':cloud_out (cloud_out msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'thickness_service)))
  'thickness_service-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'thickness_service)))
  'thickness_service-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'thickness_service)))
  "Returns string type for a service object of type '<thickness_service>"
  "phd/thickness_service")
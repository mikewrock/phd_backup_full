; Auto-generated. Do not edit!


(cl:in-package phd-srv)


;//! \htmlinclude doctor_cloud-request.msg.html

(cl:defclass <doctor_cloud-request> (roslisp-msg-protocol:ros-message)
  ((cloud_in
    :reader cloud_in
    :initarg :cloud_in
    :type sensor_msgs-msg:PointCloud2
    :initform (cl:make-instance 'sensor_msgs-msg:PointCloud2))
   (marker_file
    :reader marker_file
    :initarg :marker_file
    :type cl:string
    :initform "")
   (homing
    :reader homing
    :initarg :homing
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass doctor_cloud-request (<doctor_cloud-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <doctor_cloud-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'doctor_cloud-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name phd-srv:<doctor_cloud-request> is deprecated: use phd-srv:doctor_cloud-request instead.")))

(cl:ensure-generic-function 'cloud_in-val :lambda-list '(m))
(cl:defmethod cloud_in-val ((m <doctor_cloud-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader phd-srv:cloud_in-val is deprecated.  Use phd-srv:cloud_in instead.")
  (cloud_in m))

(cl:ensure-generic-function 'marker_file-val :lambda-list '(m))
(cl:defmethod marker_file-val ((m <doctor_cloud-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader phd-srv:marker_file-val is deprecated.  Use phd-srv:marker_file instead.")
  (marker_file m))

(cl:ensure-generic-function 'homing-val :lambda-list '(m))
(cl:defmethod homing-val ((m <doctor_cloud-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader phd-srv:homing-val is deprecated.  Use phd-srv:homing instead.")
  (homing m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <doctor_cloud-request>) ostream)
  "Serializes a message object of type '<doctor_cloud-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'cloud_in) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'marker_file))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'marker_file))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'homing) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <doctor_cloud-request>) istream)
  "Deserializes a message object of type '<doctor_cloud-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'cloud_in) istream)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'marker_file) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'marker_file) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:slot-value msg 'homing) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<doctor_cloud-request>)))
  "Returns string type for a service object of type '<doctor_cloud-request>"
  "phd/doctor_cloudRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'doctor_cloud-request)))
  "Returns string type for a service object of type 'doctor_cloud-request"
  "phd/doctor_cloudRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<doctor_cloud-request>)))
  "Returns md5sum for a message object of type '<doctor_cloud-request>"
  "d0bf3e98b4d1a486924402bbb512ae62")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'doctor_cloud-request)))
  "Returns md5sum for a message object of type 'doctor_cloud-request"
  "d0bf3e98b4d1a486924402bbb512ae62")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<doctor_cloud-request>)))
  "Returns full string definition for message of type '<doctor_cloud-request>"
  (cl:format cl:nil "sensor_msgs/PointCloud2 cloud_in~%string marker_file~%bool homing~%~%================================================================================~%MSG: sensor_msgs/PointCloud2~%# This message holds a collection of N-dimensional points, which may~%# contain additional information such as normals, intensity, etc. The~%# point data is stored as a binary blob, its layout described by the~%# contents of the \"fields\" array.~%~%# The point cloud data may be organized 2d (image-like) or 1d~%# (unordered). Point clouds organized as 2d images may be produced by~%# camera depth sensors such as stereo or time-of-flight.~%~%# Time of sensor data acquisition, and the coordinate frame ID (for 3d~%# points).~%Header header~%~%# 2D structure of the point cloud. If the cloud is unordered, height is~%# 1 and width is the length of the point cloud.~%uint32 height~%uint32 width~%~%# Describes the channels and their layout in the binary data blob.~%PointField[] fields~%~%bool    is_bigendian # Is this data bigendian?~%uint32  point_step   # Length of a point in bytes~%uint32  row_step     # Length of a row in bytes~%uint8[] data         # Actual point data, size is (row_step*height)~%~%bool is_dense        # True if there are no invalid points~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: sensor_msgs/PointField~%# This message holds the description of one point entry in the~%# PointCloud2 message format.~%uint8 INT8    = 1~%uint8 UINT8   = 2~%uint8 INT16   = 3~%uint8 UINT16  = 4~%uint8 INT32   = 5~%uint8 UINT32  = 6~%uint8 FLOAT32 = 7~%uint8 FLOAT64 = 8~%~%string name      # Name of field~%uint32 offset    # Offset from start of point struct~%uint8  datatype  # Datatype enumeration, see above~%uint32 count     # How many elements in the field~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'doctor_cloud-request)))
  "Returns full string definition for message of type 'doctor_cloud-request"
  (cl:format cl:nil "sensor_msgs/PointCloud2 cloud_in~%string marker_file~%bool homing~%~%================================================================================~%MSG: sensor_msgs/PointCloud2~%# This message holds a collection of N-dimensional points, which may~%# contain additional information such as normals, intensity, etc. The~%# point data is stored as a binary blob, its layout described by the~%# contents of the \"fields\" array.~%~%# The point cloud data may be organized 2d (image-like) or 1d~%# (unordered). Point clouds organized as 2d images may be produced by~%# camera depth sensors such as stereo or time-of-flight.~%~%# Time of sensor data acquisition, and the coordinate frame ID (for 3d~%# points).~%Header header~%~%# 2D structure of the point cloud. If the cloud is unordered, height is~%# 1 and width is the length of the point cloud.~%uint32 height~%uint32 width~%~%# Describes the channels and their layout in the binary data blob.~%PointField[] fields~%~%bool    is_bigendian # Is this data bigendian?~%uint32  point_step   # Length of a point in bytes~%uint32  row_step     # Length of a row in bytes~%uint8[] data         # Actual point data, size is (row_step*height)~%~%bool is_dense        # True if there are no invalid points~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: sensor_msgs/PointField~%# This message holds the description of one point entry in the~%# PointCloud2 message format.~%uint8 INT8    = 1~%uint8 UINT8   = 2~%uint8 INT16   = 3~%uint8 UINT16  = 4~%uint8 INT32   = 5~%uint8 UINT32  = 6~%uint8 FLOAT32 = 7~%uint8 FLOAT64 = 8~%~%string name      # Name of field~%uint32 offset    # Offset from start of point struct~%uint8  datatype  # Datatype enumeration, see above~%uint32 count     # How many elements in the field~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <doctor_cloud-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'cloud_in))
     4 (cl:length (cl:slot-value msg 'marker_file))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <doctor_cloud-request>))
  "Converts a ROS message object to a list"
  (cl:list 'doctor_cloud-request
    (cl:cons ':cloud_in (cloud_in msg))
    (cl:cons ':marker_file (marker_file msg))
    (cl:cons ':homing (homing msg))
))
;//! \htmlinclude doctor_cloud-response.msg.html

(cl:defclass <doctor_cloud-response> (roslisp-msg-protocol:ros-message)
  ((clouds
    :reader clouds
    :initarg :clouds
    :type (cl:vector phd-msg:doctor_msg)
   :initform (cl:make-array 0 :element-type 'phd-msg:doctor_msg :initial-element (cl:make-instance 'phd-msg:doctor_msg))))
)

(cl:defclass doctor_cloud-response (<doctor_cloud-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <doctor_cloud-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'doctor_cloud-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name phd-srv:<doctor_cloud-response> is deprecated: use phd-srv:doctor_cloud-response instead.")))

(cl:ensure-generic-function 'clouds-val :lambda-list '(m))
(cl:defmethod clouds-val ((m <doctor_cloud-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader phd-srv:clouds-val is deprecated.  Use phd-srv:clouds instead.")
  (clouds m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <doctor_cloud-response>) ostream)
  "Serializes a message object of type '<doctor_cloud-response>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'clouds))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'clouds))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <doctor_cloud-response>) istream)
  "Deserializes a message object of type '<doctor_cloud-response>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'clouds) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'clouds)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'phd-msg:doctor_msg))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<doctor_cloud-response>)))
  "Returns string type for a service object of type '<doctor_cloud-response>"
  "phd/doctor_cloudResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'doctor_cloud-response)))
  "Returns string type for a service object of type 'doctor_cloud-response"
  "phd/doctor_cloudResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<doctor_cloud-response>)))
  "Returns md5sum for a message object of type '<doctor_cloud-response>"
  "d0bf3e98b4d1a486924402bbb512ae62")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'doctor_cloud-response)))
  "Returns md5sum for a message object of type 'doctor_cloud-response"
  "d0bf3e98b4d1a486924402bbb512ae62")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<doctor_cloud-response>)))
  "Returns full string definition for message of type '<doctor_cloud-response>"
  (cl:format cl:nil "doctor_msg[] clouds~%~%~%================================================================================~%MSG: phd/doctor_msg~%sensor_msgs/PointCloud2 cloud_out~%float64[16] transform_mat~%float32 x~%float32 y~%float32 z~%float32 id~%~%================================================================================~%MSG: sensor_msgs/PointCloud2~%# This message holds a collection of N-dimensional points, which may~%# contain additional information such as normals, intensity, etc. The~%# point data is stored as a binary blob, its layout described by the~%# contents of the \"fields\" array.~%~%# The point cloud data may be organized 2d (image-like) or 1d~%# (unordered). Point clouds organized as 2d images may be produced by~%# camera depth sensors such as stereo or time-of-flight.~%~%# Time of sensor data acquisition, and the coordinate frame ID (for 3d~%# points).~%Header header~%~%# 2D structure of the point cloud. If the cloud is unordered, height is~%# 1 and width is the length of the point cloud.~%uint32 height~%uint32 width~%~%# Describes the channels and their layout in the binary data blob.~%PointField[] fields~%~%bool    is_bigendian # Is this data bigendian?~%uint32  point_step   # Length of a point in bytes~%uint32  row_step     # Length of a row in bytes~%uint8[] data         # Actual point data, size is (row_step*height)~%~%bool is_dense        # True if there are no invalid points~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: sensor_msgs/PointField~%# This message holds the description of one point entry in the~%# PointCloud2 message format.~%uint8 INT8    = 1~%uint8 UINT8   = 2~%uint8 INT16   = 3~%uint8 UINT16  = 4~%uint8 INT32   = 5~%uint8 UINT32  = 6~%uint8 FLOAT32 = 7~%uint8 FLOAT64 = 8~%~%string name      # Name of field~%uint32 offset    # Offset from start of point struct~%uint8  datatype  # Datatype enumeration, see above~%uint32 count     # How many elements in the field~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'doctor_cloud-response)))
  "Returns full string definition for message of type 'doctor_cloud-response"
  (cl:format cl:nil "doctor_msg[] clouds~%~%~%================================================================================~%MSG: phd/doctor_msg~%sensor_msgs/PointCloud2 cloud_out~%float64[16] transform_mat~%float32 x~%float32 y~%float32 z~%float32 id~%~%================================================================================~%MSG: sensor_msgs/PointCloud2~%# This message holds a collection of N-dimensional points, which may~%# contain additional information such as normals, intensity, etc. The~%# point data is stored as a binary blob, its layout described by the~%# contents of the \"fields\" array.~%~%# The point cloud data may be organized 2d (image-like) or 1d~%# (unordered). Point clouds organized as 2d images may be produced by~%# camera depth sensors such as stereo or time-of-flight.~%~%# Time of sensor data acquisition, and the coordinate frame ID (for 3d~%# points).~%Header header~%~%# 2D structure of the point cloud. If the cloud is unordered, height is~%# 1 and width is the length of the point cloud.~%uint32 height~%uint32 width~%~%# Describes the channels and their layout in the binary data blob.~%PointField[] fields~%~%bool    is_bigendian # Is this data bigendian?~%uint32  point_step   # Length of a point in bytes~%uint32  row_step     # Length of a row in bytes~%uint8[] data         # Actual point data, size is (row_step*height)~%~%bool is_dense        # True if there are no invalid points~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: sensor_msgs/PointField~%# This message holds the description of one point entry in the~%# PointCloud2 message format.~%uint8 INT8    = 1~%uint8 UINT8   = 2~%uint8 INT16   = 3~%uint8 UINT16  = 4~%uint8 INT32   = 5~%uint8 UINT32  = 6~%uint8 FLOAT32 = 7~%uint8 FLOAT64 = 8~%~%string name      # Name of field~%uint32 offset    # Offset from start of point struct~%uint8  datatype  # Datatype enumeration, see above~%uint32 count     # How many elements in the field~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <doctor_cloud-response>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'clouds) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <doctor_cloud-response>))
  "Converts a ROS message object to a list"
  (cl:list 'doctor_cloud-response
    (cl:cons ':clouds (clouds msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'doctor_cloud)))
  'doctor_cloud-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'doctor_cloud)))
  'doctor_cloud-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'doctor_cloud)))
  "Returns string type for a service object of type '<doctor_cloud>"
  "phd/doctor_cloud")
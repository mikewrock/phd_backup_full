; Auto-generated. Do not edit!


(cl:in-package phd-srv)


;//! \htmlinclude calc_service-request.msg.html

(cl:defclass <calc_service-request> (roslisp-msg-protocol:ros-message)
  ((pre_ids
    :reader pre_ids
    :initarg :pre_ids
    :type (cl:vector cl:integer)
   :initform (cl:make-array 0 :element-type 'cl:integer :initial-element 0))
   (post_ids
    :reader post_ids
    :initarg :post_ids
    :type (cl:vector cl:integer)
   :initform (cl:make-array 0 :element-type 'cl:integer :initial-element 0))
   (datum
    :reader datum
    :initarg :datum
    :type cl:string
    :initform "")
   (location
    :reader location
    :initarg :location
    :type cl:string
    :initform ""))
)

(cl:defclass calc_service-request (<calc_service-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <calc_service-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'calc_service-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name phd-srv:<calc_service-request> is deprecated: use phd-srv:calc_service-request instead.")))

(cl:ensure-generic-function 'pre_ids-val :lambda-list '(m))
(cl:defmethod pre_ids-val ((m <calc_service-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader phd-srv:pre_ids-val is deprecated.  Use phd-srv:pre_ids instead.")
  (pre_ids m))

(cl:ensure-generic-function 'post_ids-val :lambda-list '(m))
(cl:defmethod post_ids-val ((m <calc_service-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader phd-srv:post_ids-val is deprecated.  Use phd-srv:post_ids instead.")
  (post_ids m))

(cl:ensure-generic-function 'datum-val :lambda-list '(m))
(cl:defmethod datum-val ((m <calc_service-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader phd-srv:datum-val is deprecated.  Use phd-srv:datum instead.")
  (datum m))

(cl:ensure-generic-function 'location-val :lambda-list '(m))
(cl:defmethod location-val ((m <calc_service-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader phd-srv:location-val is deprecated.  Use phd-srv:location instead.")
  (location m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <calc_service-request>) ostream)
  "Serializes a message object of type '<calc_service-request>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'pre_ids))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    ))
   (cl:slot-value msg 'pre_ids))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'post_ids))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    ))
   (cl:slot-value msg 'post_ids))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'datum))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'datum))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'location))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'location))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <calc_service-request>) istream)
  "Deserializes a message object of type '<calc_service-request>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'pre_ids) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'pre_ids)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296)))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'post_ids) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'post_ids)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296)))))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'datum) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'datum) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'location) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'location) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<calc_service-request>)))
  "Returns string type for a service object of type '<calc_service-request>"
  "phd/calc_serviceRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'calc_service-request)))
  "Returns string type for a service object of type 'calc_service-request"
  "phd/calc_serviceRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<calc_service-request>)))
  "Returns md5sum for a message object of type '<calc_service-request>"
  "3e940f6cd215fa9d81d97d506548ddde")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'calc_service-request)))
  "Returns md5sum for a message object of type 'calc_service-request"
  "3e940f6cd215fa9d81d97d506548ddde")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<calc_service-request>)))
  "Returns full string definition for message of type '<calc_service-request>"
  (cl:format cl:nil "int32[] pre_ids~%int32[] post_ids~%string datum~%string location~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'calc_service-request)))
  "Returns full string definition for message of type 'calc_service-request"
  (cl:format cl:nil "int32[] pre_ids~%int32[] post_ids~%string datum~%string location~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <calc_service-request>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'pre_ids) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'post_ids) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:length (cl:slot-value msg 'datum))
     4 (cl:length (cl:slot-value msg 'location))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <calc_service-request>))
  "Converts a ROS message object to a list"
  (cl:list 'calc_service-request
    (cl:cons ':pre_ids (pre_ids msg))
    (cl:cons ':post_ids (post_ids msg))
    (cl:cons ':datum (datum msg))
    (cl:cons ':location (location msg))
))
;//! \htmlinclude calc_service-response.msg.html

(cl:defclass <calc_service-response> (roslisp-msg-protocol:ros-message)
  ((cloud_out
    :reader cloud_out
    :initarg :cloud_out
    :type sensor_msgs-msg:PointCloud2
    :initform (cl:make-instance 'sensor_msgs-msg:PointCloud2)))
)

(cl:defclass calc_service-response (<calc_service-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <calc_service-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'calc_service-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name phd-srv:<calc_service-response> is deprecated: use phd-srv:calc_service-response instead.")))

(cl:ensure-generic-function 'cloud_out-val :lambda-list '(m))
(cl:defmethod cloud_out-val ((m <calc_service-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader phd-srv:cloud_out-val is deprecated.  Use phd-srv:cloud_out instead.")
  (cloud_out m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <calc_service-response>) ostream)
  "Serializes a message object of type '<calc_service-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'cloud_out) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <calc_service-response>) istream)
  "Deserializes a message object of type '<calc_service-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'cloud_out) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<calc_service-response>)))
  "Returns string type for a service object of type '<calc_service-response>"
  "phd/calc_serviceResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'calc_service-response)))
  "Returns string type for a service object of type 'calc_service-response"
  "phd/calc_serviceResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<calc_service-response>)))
  "Returns md5sum for a message object of type '<calc_service-response>"
  "3e940f6cd215fa9d81d97d506548ddde")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'calc_service-response)))
  "Returns md5sum for a message object of type 'calc_service-response"
  "3e940f6cd215fa9d81d97d506548ddde")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<calc_service-response>)))
  "Returns full string definition for message of type '<calc_service-response>"
  (cl:format cl:nil "sensor_msgs/PointCloud2 cloud_out~%~%~%================================================================================~%MSG: sensor_msgs/PointCloud2~%# This message holds a collection of N-dimensional points, which may~%# contain additional information such as normals, intensity, etc. The~%# point data is stored as a binary blob, its layout described by the~%# contents of the \"fields\" array.~%~%# The point cloud data may be organized 2d (image-like) or 1d~%# (unordered). Point clouds organized as 2d images may be produced by~%# camera depth sensors such as stereo or time-of-flight.~%~%# Time of sensor data acquisition, and the coordinate frame ID (for 3d~%# points).~%Header header~%~%# 2D structure of the point cloud. If the cloud is unordered, height is~%# 1 and width is the length of the point cloud.~%uint32 height~%uint32 width~%~%# Describes the channels and their layout in the binary data blob.~%PointField[] fields~%~%bool    is_bigendian # Is this data bigendian?~%uint32  point_step   # Length of a point in bytes~%uint32  row_step     # Length of a row in bytes~%uint8[] data         # Actual point data, size is (row_step*height)~%~%bool is_dense        # True if there are no invalid points~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: sensor_msgs/PointField~%# This message holds the description of one point entry in the~%# PointCloud2 message format.~%uint8 INT8    = 1~%uint8 UINT8   = 2~%uint8 INT16   = 3~%uint8 UINT16  = 4~%uint8 INT32   = 5~%uint8 UINT32  = 6~%uint8 FLOAT32 = 7~%uint8 FLOAT64 = 8~%~%string name      # Name of field~%uint32 offset    # Offset from start of point struct~%uint8  datatype  # Datatype enumeration, see above~%uint32 count     # How many elements in the field~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'calc_service-response)))
  "Returns full string definition for message of type 'calc_service-response"
  (cl:format cl:nil "sensor_msgs/PointCloud2 cloud_out~%~%~%================================================================================~%MSG: sensor_msgs/PointCloud2~%# This message holds a collection of N-dimensional points, which may~%# contain additional information such as normals, intensity, etc. The~%# point data is stored as a binary blob, its layout described by the~%# contents of the \"fields\" array.~%~%# The point cloud data may be organized 2d (image-like) or 1d~%# (unordered). Point clouds organized as 2d images may be produced by~%# camera depth sensors such as stereo or time-of-flight.~%~%# Time of sensor data acquisition, and the coordinate frame ID (for 3d~%# points).~%Header header~%~%# 2D structure of the point cloud. If the cloud is unordered, height is~%# 1 and width is the length of the point cloud.~%uint32 height~%uint32 width~%~%# Describes the channels and their layout in the binary data blob.~%PointField[] fields~%~%bool    is_bigendian # Is this data bigendian?~%uint32  point_step   # Length of a point in bytes~%uint32  row_step     # Length of a row in bytes~%uint8[] data         # Actual point data, size is (row_step*height)~%~%bool is_dense        # True if there are no invalid points~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: sensor_msgs/PointField~%# This message holds the description of one point entry in the~%# PointCloud2 message format.~%uint8 INT8    = 1~%uint8 UINT8   = 2~%uint8 INT16   = 3~%uint8 UINT16  = 4~%uint8 INT32   = 5~%uint8 UINT32  = 6~%uint8 FLOAT32 = 7~%uint8 FLOAT64 = 8~%~%string name      # Name of field~%uint32 offset    # Offset from start of point struct~%uint8  datatype  # Datatype enumeration, see above~%uint32 count     # How many elements in the field~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <calc_service-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'cloud_out))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <calc_service-response>))
  "Converts a ROS message object to a list"
  (cl:list 'calc_service-response
    (cl:cons ':cloud_out (cloud_out msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'calc_service)))
  'calc_service-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'calc_service)))
  'calc_service-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'calc_service)))
  "Returns string type for a service object of type '<calc_service>"
  "phd/calc_service")
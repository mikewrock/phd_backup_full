# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from phd/localize_cloudRequest.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import std_msgs.msg
import sensor_msgs.msg

class localize_cloudRequest(genpy.Message):
  _md5sum = "4177966bfe648dd616186ef1a0d7924a"
  _type = "phd/localize_cloudRequest"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """sensor_msgs/PointCloud2 cloud_in
string marker_file
bool homing

================================================================================
MSG: sensor_msgs/PointCloud2
# This message holds a collection of N-dimensional points, which may
# contain additional information such as normals, intensity, etc. The
# point data is stored as a binary blob, its layout described by the
# contents of the "fields" array.

# The point cloud data may be organized 2d (image-like) or 1d
# (unordered). Point clouds organized as 2d images may be produced by
# camera depth sensors such as stereo or time-of-flight.

# Time of sensor data acquisition, and the coordinate frame ID (for 3d
# points).
Header header

# 2D structure of the point cloud. If the cloud is unordered, height is
# 1 and width is the length of the point cloud.
uint32 height
uint32 width

# Describes the channels and their layout in the binary data blob.
PointField[] fields

bool    is_bigendian # Is this data bigendian?
uint32  point_step   # Length of a point in bytes
uint32  row_step     # Length of a row in bytes
uint8[] data         # Actual point data, size is (row_step*height)

bool is_dense        # True if there are no invalid points

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

================================================================================
MSG: sensor_msgs/PointField
# This message holds the description of one point entry in the
# PointCloud2 message format.
uint8 INT8    = 1
uint8 UINT8   = 2
uint8 INT16   = 3
uint8 UINT16  = 4
uint8 INT32   = 5
uint8 UINT32  = 6
uint8 FLOAT32 = 7
uint8 FLOAT64 = 8

string name      # Name of field
uint32 offset    # Offset from start of point struct
uint8  datatype  # Datatype enumeration, see above
uint32 count     # How many elements in the field
"""
  __slots__ = ['cloud_in','marker_file','homing']
  _slot_types = ['sensor_msgs/PointCloud2','string','bool']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       cloud_in,marker_file,homing

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(localize_cloudRequest, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.cloud_in is None:
        self.cloud_in = sensor_msgs.msg.PointCloud2()
      if self.marker_file is None:
        self.marker_file = ''
      if self.homing is None:
        self.homing = False
    else:
      self.cloud_in = sensor_msgs.msg.PointCloud2()
      self.marker_file = ''
      self.homing = False

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self
      buff.write(_struct_3I.pack(_x.cloud_in.header.seq, _x.cloud_in.header.stamp.secs, _x.cloud_in.header.stamp.nsecs))
      _x = self.cloud_in.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      if python3:
        buff.write(struct.pack('<I%sB'%length, length, *_x))
      else:
        buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_2I.pack(_x.cloud_in.height, _x.cloud_in.width))
      length = len(self.cloud_in.fields)
      buff.write(_struct_I.pack(length))
      for val1 in self.cloud_in.fields:
        _x = val1.name
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        if python3:
          buff.write(struct.pack('<I%sB'%length, length, *_x))
        else:
          buff.write(struct.pack('<I%ss'%length, length, _x))
        _x = val1
        buff.write(_struct_IBI.pack(_x.offset, _x.datatype, _x.count))
      _x = self
      buff.write(_struct_B2I.pack(_x.cloud_in.is_bigendian, _x.cloud_in.point_step, _x.cloud_in.row_step))
      _x = self.cloud_in.data
      length = len(_x)
      # - if encoded as a list instead, serialize as bytes instead of string
      if type(_x) in [list, tuple]:
        buff.write(struct.pack('<I%sB'%length, length, *_x))
      else:
        buff.write(struct.pack('<I%ss'%length, length, _x))
      buff.write(_struct_B.pack(self.cloud_in.is_dense))
      _x = self.marker_file
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      if python3:
        buff.write(struct.pack('<I%sB'%length, length, *_x))
      else:
        buff.write(struct.pack('<I%ss'%length, length, _x))
      buff.write(_struct_B.pack(self.homing))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.cloud_in is None:
        self.cloud_in = sensor_msgs.msg.PointCloud2()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.cloud_in.header.seq, _x.cloud_in.header.stamp.secs, _x.cloud_in.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.cloud_in.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.cloud_in.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 8
      (_x.cloud_in.height, _x.cloud_in.width,) = _struct_2I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.cloud_in.fields = []
      for i in range(0, length):
        val1 = sensor_msgs.msg.PointField()
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1.name = str[start:end].decode('utf-8')
        else:
          val1.name = str[start:end]
        _x = val1
        start = end
        end += 9
        (_x.offset, _x.datatype, _x.count,) = _struct_IBI.unpack(str[start:end])
        self.cloud_in.fields.append(val1)
      _x = self
      start = end
      end += 9
      (_x.cloud_in.is_bigendian, _x.cloud_in.point_step, _x.cloud_in.row_step,) = _struct_B2I.unpack(str[start:end])
      self.cloud_in.is_bigendian = bool(self.cloud_in.is_bigendian)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      self.cloud_in.data = str[start:end]
      start = end
      end += 1
      (self.cloud_in.is_dense,) = _struct_B.unpack(str[start:end])
      self.cloud_in.is_dense = bool(self.cloud_in.is_dense)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.marker_file = str[start:end].decode('utf-8')
      else:
        self.marker_file = str[start:end]
      start = end
      end += 1
      (self.homing,) = _struct_B.unpack(str[start:end])
      self.homing = bool(self.homing)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_struct_3I.pack(_x.cloud_in.header.seq, _x.cloud_in.header.stamp.secs, _x.cloud_in.header.stamp.nsecs))
      _x = self.cloud_in.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      if python3:
        buff.write(struct.pack('<I%sB'%length, length, *_x))
      else:
        buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_2I.pack(_x.cloud_in.height, _x.cloud_in.width))
      length = len(self.cloud_in.fields)
      buff.write(_struct_I.pack(length))
      for val1 in self.cloud_in.fields:
        _x = val1.name
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        if python3:
          buff.write(struct.pack('<I%sB'%length, length, *_x))
        else:
          buff.write(struct.pack('<I%ss'%length, length, _x))
        _x = val1
        buff.write(_struct_IBI.pack(_x.offset, _x.datatype, _x.count))
      _x = self
      buff.write(_struct_B2I.pack(_x.cloud_in.is_bigendian, _x.cloud_in.point_step, _x.cloud_in.row_step))
      _x = self.cloud_in.data
      length = len(_x)
      # - if encoded as a list instead, serialize as bytes instead of string
      if type(_x) in [list, tuple]:
        buff.write(struct.pack('<I%sB'%length, length, *_x))
      else:
        buff.write(struct.pack('<I%ss'%length, length, _x))
      buff.write(_struct_B.pack(self.cloud_in.is_dense))
      _x = self.marker_file
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      if python3:
        buff.write(struct.pack('<I%sB'%length, length, *_x))
      else:
        buff.write(struct.pack('<I%ss'%length, length, _x))
      buff.write(_struct_B.pack(self.homing))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.cloud_in is None:
        self.cloud_in = sensor_msgs.msg.PointCloud2()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.cloud_in.header.seq, _x.cloud_in.header.stamp.secs, _x.cloud_in.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.cloud_in.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.cloud_in.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 8
      (_x.cloud_in.height, _x.cloud_in.width,) = _struct_2I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.cloud_in.fields = []
      for i in range(0, length):
        val1 = sensor_msgs.msg.PointField()
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1.name = str[start:end].decode('utf-8')
        else:
          val1.name = str[start:end]
        _x = val1
        start = end
        end += 9
        (_x.offset, _x.datatype, _x.count,) = _struct_IBI.unpack(str[start:end])
        self.cloud_in.fields.append(val1)
      _x = self
      start = end
      end += 9
      (_x.cloud_in.is_bigendian, _x.cloud_in.point_step, _x.cloud_in.row_step,) = _struct_B2I.unpack(str[start:end])
      self.cloud_in.is_bigendian = bool(self.cloud_in.is_bigendian)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      self.cloud_in.data = str[start:end]
      start = end
      end += 1
      (self.cloud_in.is_dense,) = _struct_B.unpack(str[start:end])
      self.cloud_in.is_dense = bool(self.cloud_in.is_dense)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.marker_file = str[start:end].decode('utf-8')
      else:
        self.marker_file = str[start:end]
      start = end
      end += 1
      (self.homing,) = _struct_B.unpack(str[start:end])
      self.homing = bool(self.homing)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_IBI = struct.Struct("<IBI")
_struct_3I = struct.Struct("<3I")
_struct_B = struct.Struct("<B")
_struct_2I = struct.Struct("<2I")
_struct_B2I = struct.Struct("<B2I")
# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from phd/localize_cloudResponse.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import std_msgs.msg
import sensor_msgs.msg

class localize_cloudResponse(genpy.Message):
  _md5sum = "b199ee0f9f915c62760e2e9b5c864faa"
  _type = "phd/localize_cloudResponse"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """sensor_msgs/PointCloud2 cloud_out
float64[16] transform_mat


================================================================================
MSG: sensor_msgs/PointCloud2
# This message holds a collection of N-dimensional points, which may
# contain additional information such as normals, intensity, etc. The
# point data is stored as a binary blob, its layout described by the
# contents of the "fields" array.

# The point cloud data may be organized 2d (image-like) or 1d
# (unordered). Point clouds organized as 2d images may be produced by
# camera depth sensors such as stereo or time-of-flight.

# Time of sensor data acquisition, and the coordinate frame ID (for 3d
# points).
Header header

# 2D structure of the point cloud. If the cloud is unordered, height is
# 1 and width is the length of the point cloud.
uint32 height
uint32 width

# Describes the channels and their layout in the binary data blob.
PointField[] fields

bool    is_bigendian # Is this data bigendian?
uint32  point_step   # Length of a point in bytes
uint32  row_step     # Length of a row in bytes
uint8[] data         # Actual point data, size is (row_step*height)

bool is_dense        # True if there are no invalid points

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

================================================================================
MSG: sensor_msgs/PointField
# This message holds the description of one point entry in the
# PointCloud2 message format.
uint8 INT8    = 1
uint8 UINT8   = 2
uint8 INT16   = 3
uint8 UINT16  = 4
uint8 INT32   = 5
uint8 UINT32  = 6
uint8 FLOAT32 = 7
uint8 FLOAT64 = 8

string name      # Name of field
uint32 offset    # Offset from start of point struct
uint8  datatype  # Datatype enumeration, see above
uint32 count     # How many elements in the field
"""
  __slots__ = ['cloud_out','transform_mat']
  _slot_types = ['sensor_msgs/PointCloud2','float64[16]']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       cloud_out,transform_mat

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(localize_cloudResponse, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.cloud_out is None:
        self.cloud_out = sensor_msgs.msg.PointCloud2()
      if self.transform_mat is None:
        self.transform_mat = [0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.]
    else:
      self.cloud_out = sensor_msgs.msg.PointCloud2()
      self.transform_mat = [0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.]

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self
      buff.write(_struct_3I.pack(_x.cloud_out.header.seq, _x.cloud_out.header.stamp.secs, _x.cloud_out.header.stamp.nsecs))
      _x = self.cloud_out.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      if python3:
        buff.write(struct.pack('<I%sB'%length, length, *_x))
      else:
        buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_2I.pack(_x.cloud_out.height, _x.cloud_out.width))
      length = len(self.cloud_out.fields)
      buff.write(_struct_I.pack(length))
      for val1 in self.cloud_out.fields:
        _x = val1.name
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        if python3:
          buff.write(struct.pack('<I%sB'%length, length, *_x))
        else:
          buff.write(struct.pack('<I%ss'%length, length, _x))
        _x = val1
        buff.write(_struct_IBI.pack(_x.offset, _x.datatype, _x.count))
      _x = self
      buff.write(_struct_B2I.pack(_x.cloud_out.is_bigendian, _x.cloud_out.point_step, _x.cloud_out.row_step))
      _x = self.cloud_out.data
      length = len(_x)
      # - if encoded as a list instead, serialize as bytes instead of string
      if type(_x) in [list, tuple]:
        buff.write(struct.pack('<I%sB'%length, length, *_x))
      else:
        buff.write(struct.pack('<I%ss'%length, length, _x))
      buff.write(_struct_B.pack(self.cloud_out.is_dense))
      buff.write(_struct_16d.pack(*self.transform_mat))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.cloud_out is None:
        self.cloud_out = sensor_msgs.msg.PointCloud2()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.cloud_out.header.seq, _x.cloud_out.header.stamp.secs, _x.cloud_out.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.cloud_out.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.cloud_out.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 8
      (_x.cloud_out.height, _x.cloud_out.width,) = _struct_2I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.cloud_out.fields = []
      for i in range(0, length):
        val1 = sensor_msgs.msg.PointField()
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1.name = str[start:end].decode('utf-8')
        else:
          val1.name = str[start:end]
        _x = val1
        start = end
        end += 9
        (_x.offset, _x.datatype, _x.count,) = _struct_IBI.unpack(str[start:end])
        self.cloud_out.fields.append(val1)
      _x = self
      start = end
      end += 9
      (_x.cloud_out.is_bigendian, _x.cloud_out.point_step, _x.cloud_out.row_step,) = _struct_B2I.unpack(str[start:end])
      self.cloud_out.is_bigendian = bool(self.cloud_out.is_bigendian)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      self.cloud_out.data = str[start:end]
      start = end
      end += 1
      (self.cloud_out.is_dense,) = _struct_B.unpack(str[start:end])
      self.cloud_out.is_dense = bool(self.cloud_out.is_dense)
      start = end
      end += 128
      self.transform_mat = _struct_16d.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_struct_3I.pack(_x.cloud_out.header.seq, _x.cloud_out.header.stamp.secs, _x.cloud_out.header.stamp.nsecs))
      _x = self.cloud_out.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      if python3:
        buff.write(struct.pack('<I%sB'%length, length, *_x))
      else:
        buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_2I.pack(_x.cloud_out.height, _x.cloud_out.width))
      length = len(self.cloud_out.fields)
      buff.write(_struct_I.pack(length))
      for val1 in self.cloud_out.fields:
        _x = val1.name
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        if python3:
          buff.write(struct.pack('<I%sB'%length, length, *_x))
        else:
          buff.write(struct.pack('<I%ss'%length, length, _x))
        _x = val1
        buff.write(_struct_IBI.pack(_x.offset, _x.datatype, _x.count))
      _x = self
      buff.write(_struct_B2I.pack(_x.cloud_out.is_bigendian, _x.cloud_out.point_step, _x.cloud_out.row_step))
      _x = self.cloud_out.data
      length = len(_x)
      # - if encoded as a list instead, serialize as bytes instead of string
      if type(_x) in [list, tuple]:
        buff.write(struct.pack('<I%sB'%length, length, *_x))
      else:
        buff.write(struct.pack('<I%ss'%length, length, _x))
      buff.write(_struct_B.pack(self.cloud_out.is_dense))
      buff.write(self.transform_mat.tostring())
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.cloud_out is None:
        self.cloud_out = sensor_msgs.msg.PointCloud2()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.cloud_out.header.seq, _x.cloud_out.header.stamp.secs, _x.cloud_out.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.cloud_out.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.cloud_out.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 8
      (_x.cloud_out.height, _x.cloud_out.width,) = _struct_2I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.cloud_out.fields = []
      for i in range(0, length):
        val1 = sensor_msgs.msg.PointField()
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1.name = str[start:end].decode('utf-8')
        else:
          val1.name = str[start:end]
        _x = val1
        start = end
        end += 9
        (_x.offset, _x.datatype, _x.count,) = _struct_IBI.unpack(str[start:end])
        self.cloud_out.fields.append(val1)
      _x = self
      start = end
      end += 9
      (_x.cloud_out.is_bigendian, _x.cloud_out.point_step, _x.cloud_out.row_step,) = _struct_B2I.unpack(str[start:end])
      self.cloud_out.is_bigendian = bool(self.cloud_out.is_bigendian)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      self.cloud_out.data = str[start:end]
      start = end
      end += 1
      (self.cloud_out.is_dense,) = _struct_B.unpack(str[start:end])
      self.cloud_out.is_dense = bool(self.cloud_out.is_dense)
      start = end
      end += 128
      self.transform_mat = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=16)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_IBI = struct.Struct("<IBI")
_struct_B = struct.Struct("<B")
_struct_16d = struct.Struct("<16d")
_struct_3I = struct.Struct("<3I")
_struct_B2I = struct.Struct("<B2I")
_struct_2I = struct.Struct("<2I")
class localize_cloud(object):
  _type          = 'phd/localize_cloud'
  _md5sum = 'e94e341fd35037c227d8abedb9ee2c51'
  _request_class  = localize_cloudRequest
  _response_class = localize_cloudResponse

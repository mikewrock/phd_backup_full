# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from phd/marker_msg.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import geometry_msgs.msg

class marker_msg(genpy.Message):
  _md5sum = "46163b1a11ff6053bb2e316cbd0d9ada"
  _type = "phd/marker_msg"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """geometry_msgs/Point p1
geometry_msgs/Point p2
geometry_msgs/Point p3
geometry_msgs/Point vec1
geometry_msgs/Point vec2
float64[] transform
float32 VAL1
float32 VAL2
float32 VAL3
float32 VAL4

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
"""
  __slots__ = ['p1','p2','p3','vec1','vec2','transform','VAL1','VAL2','VAL3','VAL4']
  _slot_types = ['geometry_msgs/Point','geometry_msgs/Point','geometry_msgs/Point','geometry_msgs/Point','geometry_msgs/Point','float64[]','float32','float32','float32','float32']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       p1,p2,p3,vec1,vec2,transform,VAL1,VAL2,VAL3,VAL4

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(marker_msg, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.p1 is None:
        self.p1 = geometry_msgs.msg.Point()
      if self.p2 is None:
        self.p2 = geometry_msgs.msg.Point()
      if self.p3 is None:
        self.p3 = geometry_msgs.msg.Point()
      if self.vec1 is None:
        self.vec1 = geometry_msgs.msg.Point()
      if self.vec2 is None:
        self.vec2 = geometry_msgs.msg.Point()
      if self.transform is None:
        self.transform = []
      if self.VAL1 is None:
        self.VAL1 = 0.
      if self.VAL2 is None:
        self.VAL2 = 0.
      if self.VAL3 is None:
        self.VAL3 = 0.
      if self.VAL4 is None:
        self.VAL4 = 0.
    else:
      self.p1 = geometry_msgs.msg.Point()
      self.p2 = geometry_msgs.msg.Point()
      self.p3 = geometry_msgs.msg.Point()
      self.vec1 = geometry_msgs.msg.Point()
      self.vec2 = geometry_msgs.msg.Point()
      self.transform = []
      self.VAL1 = 0.
      self.VAL2 = 0.
      self.VAL3 = 0.
      self.VAL4 = 0.

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
      buff.write(_struct_15d.pack(_x.p1.x, _x.p1.y, _x.p1.z, _x.p2.x, _x.p2.y, _x.p2.z, _x.p3.x, _x.p3.y, _x.p3.z, _x.vec1.x, _x.vec1.y, _x.vec1.z, _x.vec2.x, _x.vec2.y, _x.vec2.z))
      length = len(self.transform)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(struct.pack(pattern, *self.transform))
      _x = self
      buff.write(_struct_4f.pack(_x.VAL1, _x.VAL2, _x.VAL3, _x.VAL4))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.p1 is None:
        self.p1 = geometry_msgs.msg.Point()
      if self.p2 is None:
        self.p2 = geometry_msgs.msg.Point()
      if self.p3 is None:
        self.p3 = geometry_msgs.msg.Point()
      if self.vec1 is None:
        self.vec1 = geometry_msgs.msg.Point()
      if self.vec2 is None:
        self.vec2 = geometry_msgs.msg.Point()
      end = 0
      _x = self
      start = end
      end += 120
      (_x.p1.x, _x.p1.y, _x.p1.z, _x.p2.x, _x.p2.y, _x.p2.z, _x.p3.x, _x.p3.y, _x.p3.z, _x.vec1.x, _x.vec1.y, _x.vec1.z, _x.vec2.x, _x.vec2.y, _x.vec2.z,) = _struct_15d.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.transform = struct.unpack(pattern, str[start:end])
      _x = self
      start = end
      end += 16
      (_x.VAL1, _x.VAL2, _x.VAL3, _x.VAL4,) = _struct_4f.unpack(str[start:end])
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
      buff.write(_struct_15d.pack(_x.p1.x, _x.p1.y, _x.p1.z, _x.p2.x, _x.p2.y, _x.p2.z, _x.p3.x, _x.p3.y, _x.p3.z, _x.vec1.x, _x.vec1.y, _x.vec1.z, _x.vec2.x, _x.vec2.y, _x.vec2.z))
      length = len(self.transform)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(self.transform.tostring())
      _x = self
      buff.write(_struct_4f.pack(_x.VAL1, _x.VAL2, _x.VAL3, _x.VAL4))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.p1 is None:
        self.p1 = geometry_msgs.msg.Point()
      if self.p2 is None:
        self.p2 = geometry_msgs.msg.Point()
      if self.p3 is None:
        self.p3 = geometry_msgs.msg.Point()
      if self.vec1 is None:
        self.vec1 = geometry_msgs.msg.Point()
      if self.vec2 is None:
        self.vec2 = geometry_msgs.msg.Point()
      end = 0
      _x = self
      start = end
      end += 120
      (_x.p1.x, _x.p1.y, _x.p1.z, _x.p2.x, _x.p2.y, _x.p2.z, _x.p3.x, _x.p3.y, _x.p3.z, _x.vec1.x, _x.vec1.y, _x.vec1.z, _x.vec2.x, _x.vec2.y, _x.vec2.z,) = _struct_15d.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.transform = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=length)
      _x = self
      start = end
      end += 16
      (_x.VAL1, _x.VAL2, _x.VAL3, _x.VAL4,) = _struct_4f.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_4f = struct.Struct("<4f")
_struct_15d = struct.Struct("<15d")

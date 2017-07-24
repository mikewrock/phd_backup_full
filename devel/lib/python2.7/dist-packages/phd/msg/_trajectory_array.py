# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from phd/trajectory_array.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import phd.msg

class trajectory_array(genpy.Message):
  _md5sum = "a2dbeff104317944dceb55b36edff5f0"
  _type = "phd/trajectory_array"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """trajectory_msg[] sections


================================================================================
MSG: phd/trajectory_msg
trajectory_point[] points
trajectory_point start_pt

================================================================================
MSG: phd/trajectory_point
float32 x
float32 y
float32 z
float32 nx
float32 ny
float32 nz
float32 d
float32 d_abs

"""
  __slots__ = ['sections']
  _slot_types = ['phd/trajectory_msg[]']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       sections

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(trajectory_array, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.sections is None:
        self.sections = []
    else:
      self.sections = []

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
      length = len(self.sections)
      buff.write(_struct_I.pack(length))
      for val1 in self.sections:
        length = len(val1.points)
        buff.write(_struct_I.pack(length))
        for val2 in val1.points:
          _x = val2
          buff.write(_struct_8f.pack(_x.x, _x.y, _x.z, _x.nx, _x.ny, _x.nz, _x.d, _x.d_abs))
        _v1 = val1.start_pt
        _x = _v1
        buff.write(_struct_8f.pack(_x.x, _x.y, _x.z, _x.nx, _x.ny, _x.nz, _x.d, _x.d_abs))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.sections is None:
        self.sections = None
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.sections = []
      for i in range(0, length):
        val1 = phd.msg.trajectory_msg()
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        val1.points = []
        for i in range(0, length):
          val2 = phd.msg.trajectory_point()
          _x = val2
          start = end
          end += 32
          (_x.x, _x.y, _x.z, _x.nx, _x.ny, _x.nz, _x.d, _x.d_abs,) = _struct_8f.unpack(str[start:end])
          val1.points.append(val2)
        _v2 = val1.start_pt
        _x = _v2
        start = end
        end += 32
        (_x.x, _x.y, _x.z, _x.nx, _x.ny, _x.nz, _x.d, _x.d_abs,) = _struct_8f.unpack(str[start:end])
        self.sections.append(val1)
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
      length = len(self.sections)
      buff.write(_struct_I.pack(length))
      for val1 in self.sections:
        length = len(val1.points)
        buff.write(_struct_I.pack(length))
        for val2 in val1.points:
          _x = val2
          buff.write(_struct_8f.pack(_x.x, _x.y, _x.z, _x.nx, _x.ny, _x.nz, _x.d, _x.d_abs))
        _v3 = val1.start_pt
        _x = _v3
        buff.write(_struct_8f.pack(_x.x, _x.y, _x.z, _x.nx, _x.ny, _x.nz, _x.d, _x.d_abs))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.sections is None:
        self.sections = None
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.sections = []
      for i in range(0, length):
        val1 = phd.msg.trajectory_msg()
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        val1.points = []
        for i in range(0, length):
          val2 = phd.msg.trajectory_point()
          _x = val2
          start = end
          end += 32
          (_x.x, _x.y, _x.z, _x.nx, _x.ny, _x.nz, _x.d, _x.d_abs,) = _struct_8f.unpack(str[start:end])
          val1.points.append(val2)
        _v4 = val1.start_pt
        _x = _v4
        start = end
        end += 32
        (_x.x, _x.y, _x.z, _x.nx, _x.ny, _x.nz, _x.d, _x.d_abs,) = _struct_8f.unpack(str[start:end])
        self.sections.append(val1)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_8f = struct.Struct("<8f")

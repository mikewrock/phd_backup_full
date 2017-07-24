# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from wrock/cube_msg.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class cube_msg(genpy.Message):
  _md5sum = "8b256f374af6530303e5be6f09f99d1e"
  _type = "wrock/cube_msg"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """float32 j1
float32 j2
float32 j3
float32 j4
float32 j5
float32 vel
float32 acc
bool pose

"""
  __slots__ = ['j1','j2','j3','j4','j5','vel','acc','pose']
  _slot_types = ['float32','float32','float32','float32','float32','float32','float32','bool']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       j1,j2,j3,j4,j5,vel,acc,pose

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(cube_msg, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.j1 is None:
        self.j1 = 0.
      if self.j2 is None:
        self.j2 = 0.
      if self.j3 is None:
        self.j3 = 0.
      if self.j4 is None:
        self.j4 = 0.
      if self.j5 is None:
        self.j5 = 0.
      if self.vel is None:
        self.vel = 0.
      if self.acc is None:
        self.acc = 0.
      if self.pose is None:
        self.pose = False
    else:
      self.j1 = 0.
      self.j2 = 0.
      self.j3 = 0.
      self.j4 = 0.
      self.j5 = 0.
      self.vel = 0.
      self.acc = 0.
      self.pose = False

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
      buff.write(_struct_7fB.pack(_x.j1, _x.j2, _x.j3, _x.j4, _x.j5, _x.vel, _x.acc, _x.pose))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      end = 0
      _x = self
      start = end
      end += 29
      (_x.j1, _x.j2, _x.j3, _x.j4, _x.j5, _x.vel, _x.acc, _x.pose,) = _struct_7fB.unpack(str[start:end])
      self.pose = bool(self.pose)
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
      buff.write(_struct_7fB.pack(_x.j1, _x.j2, _x.j3, _x.j4, _x.j5, _x.vel, _x.acc, _x.pose))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      end = 0
      _x = self
      start = end
      end += 29
      (_x.j1, _x.j2, _x.j3, _x.j4, _x.j5, _x.vel, _x.acc, _x.pose,) = _struct_7fB.unpack(str[start:end])
      self.pose = bool(self.pose)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_7fB = struct.Struct("<7fB")

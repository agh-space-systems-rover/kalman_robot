# generated from rosidl_generator_py/resource/_idl.py.em
# with input from kalman_interfaces:msg/GpsTag.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_GpsTag(type):
    """Metaclass of message 'GpsTag'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('kalman_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'kalman_interfaces.msg.GpsTag')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__gps_tag
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__gps_tag
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__gps_tag
            cls._TYPE_SUPPORT = module.type_support_msg__msg__gps_tag
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__gps_tag

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class GpsTag(metaclass=Metaclass_GpsTag):
    """Message class 'GpsTag'."""

    __slots__ = [
        '_tag_id',
        '_tag_lat',
        '_tag_lon',
    ]

    _fields_and_field_types = {
        'tag_id': 'int8',
        'tag_lat': 'float',
        'tag_lon': 'float',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('int8'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.tag_id = kwargs.get('tag_id', int())
        self.tag_lat = kwargs.get('tag_lat', float())
        self.tag_lon = kwargs.get('tag_lon', float())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.tag_id != other.tag_id:
            return False
        if self.tag_lat != other.tag_lat:
            return False
        if self.tag_lon != other.tag_lon:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def tag_id(self):
        """Message field 'tag_id'."""
        return self._tag_id

    @tag_id.setter
    def tag_id(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'tag_id' field must be of type 'int'"
            assert value >= -128 and value < 128, \
                "The 'tag_id' field must be an integer in [-128, 127]"
        self._tag_id = value

    @builtins.property
    def tag_lat(self):
        """Message field 'tag_lat'."""
        return self._tag_lat

    @tag_lat.setter
    def tag_lat(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'tag_lat' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'tag_lat' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._tag_lat = value

    @builtins.property
    def tag_lon(self):
        """Message field 'tag_lon'."""
        return self._tag_lon

    @tag_lon.setter
    def tag_lon(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'tag_lon' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'tag_lon' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._tag_lon = value

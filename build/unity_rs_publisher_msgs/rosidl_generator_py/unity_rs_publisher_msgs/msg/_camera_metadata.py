# generated from rosidl_generator_py/resource/_idl.py.em
# with input from unity_rs_publisher_msgs:msg/CameraMetadata.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_CameraMetadata(type):
    """Metaclass of message 'CameraMetadata'."""

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
            module = import_type_support('unity_rs_publisher_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'unity_rs_publisher_msgs.msg.CameraMetadata')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__camera_metadata
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__camera_metadata
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__camera_metadata
            cls._TYPE_SUPPORT = module.type_support_msg__msg__camera_metadata
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__camera_metadata

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class CameraMetadata(metaclass=Metaclass_CameraMetadata):
    """Message class 'CameraMetadata'."""

    __slots__ = [
        '_width',
        '_height',
        '_depth_min',
        '_depth_max',
        '_fx',
        '_fy',
        '_cx',
        '_cy',
    ]

    _fields_and_field_types = {
        'width': 'uint32',
        'height': 'uint32',
        'depth_min': 'float',
        'depth_max': 'float',
        'fx': 'float',
        'fy': 'float',
        'cx': 'float',
        'cy': 'float',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.width = kwargs.get('width', int())
        self.height = kwargs.get('height', int())
        self.depth_min = kwargs.get('depth_min', float())
        self.depth_max = kwargs.get('depth_max', float())
        self.fx = kwargs.get('fx', float())
        self.fy = kwargs.get('fy', float())
        self.cx = kwargs.get('cx', float())
        self.cy = kwargs.get('cy', float())

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
        if self.width != other.width:
            return False
        if self.height != other.height:
            return False
        if self.depth_min != other.depth_min:
            return False
        if self.depth_max != other.depth_max:
            return False
        if self.fx != other.fx:
            return False
        if self.fy != other.fy:
            return False
        if self.cx != other.cx:
            return False
        if self.cy != other.cy:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def width(self):
        """Message field 'width'."""
        return self._width

    @width.setter
    def width(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'width' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'width' field must be an unsigned integer in [0, 4294967295]"
        self._width = value

    @builtins.property
    def height(self):
        """Message field 'height'."""
        return self._height

    @height.setter
    def height(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'height' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'height' field must be an unsigned integer in [0, 4294967295]"
        self._height = value

    @builtins.property
    def depth_min(self):
        """Message field 'depth_min'."""
        return self._depth_min

    @depth_min.setter
    def depth_min(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'depth_min' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'depth_min' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._depth_min = value

    @builtins.property
    def depth_max(self):
        """Message field 'depth_max'."""
        return self._depth_max

    @depth_max.setter
    def depth_max(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'depth_max' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'depth_max' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._depth_max = value

    @builtins.property
    def fx(self):
        """Message field 'fx'."""
        return self._fx

    @fx.setter
    def fx(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'fx' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'fx' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._fx = value

    @builtins.property
    def fy(self):
        """Message field 'fy'."""
        return self._fy

    @fy.setter
    def fy(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'fy' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'fy' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._fy = value

    @builtins.property
    def cx(self):
        """Message field 'cx'."""
        return self._cx

    @cx.setter
    def cx(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'cx' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'cx' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._cx = value

    @builtins.property
    def cy(self):
        """Message field 'cy'."""
        return self._cy

    @cy.setter
    def cy(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'cy' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'cy' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._cy = value

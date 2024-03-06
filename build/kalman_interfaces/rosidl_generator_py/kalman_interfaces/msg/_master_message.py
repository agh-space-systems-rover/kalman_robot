# generated from rosidl_generator_py/resource/_idl.py.em
# with input from kalman_interfaces:msg/MasterMessage.idl
# generated code does not contain a copyright notice


# Import statements for member types

# Member 'data'
import array  # noqa: E402, I100

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_MasterMessage(type):
    """Metaclass of message 'MasterMessage'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'UEUOS_SET_STATE': 96,
        'UEUOS_SET_COLOR': 97,
        'UEUOS_SET_EFFECT': 98,
        'MOTOR_SET_WHEELS': 64,
        'AUTONOMY_SWITCH': 32,
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
                'kalman_interfaces.msg.MasterMessage')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__master_message
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__master_message
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__master_message
            cls._TYPE_SUPPORT = module.type_support_msg__msg__master_message
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__master_message

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'UEUOS_SET_STATE': cls.__constants['UEUOS_SET_STATE'],
            'UEUOS_SET_COLOR': cls.__constants['UEUOS_SET_COLOR'],
            'UEUOS_SET_EFFECT': cls.__constants['UEUOS_SET_EFFECT'],
            'MOTOR_SET_WHEELS': cls.__constants['MOTOR_SET_WHEELS'],
            'AUTONOMY_SWITCH': cls.__constants['AUTONOMY_SWITCH'],
        }

    @property
    def UEUOS_SET_STATE(self):
        """Message constant 'UEUOS_SET_STATE'."""
        return Metaclass_MasterMessage.__constants['UEUOS_SET_STATE']

    @property
    def UEUOS_SET_COLOR(self):
        """Message constant 'UEUOS_SET_COLOR'."""
        return Metaclass_MasterMessage.__constants['UEUOS_SET_COLOR']

    @property
    def UEUOS_SET_EFFECT(self):
        """Message constant 'UEUOS_SET_EFFECT'."""
        return Metaclass_MasterMessage.__constants['UEUOS_SET_EFFECT']

    @property
    def MOTOR_SET_WHEELS(self):
        """Message constant 'MOTOR_SET_WHEELS'."""
        return Metaclass_MasterMessage.__constants['MOTOR_SET_WHEELS']

    @property
    def AUTONOMY_SWITCH(self):
        """Message constant 'AUTONOMY_SWITCH'."""
        return Metaclass_MasterMessage.__constants['AUTONOMY_SWITCH']


class MasterMessage(metaclass=Metaclass_MasterMessage):
    """
    Message class 'MasterMessage'.

    Constants:
      UEUOS_SET_STATE
      UEUOS_SET_COLOR
      UEUOS_SET_EFFECT
      MOTOR_SET_WHEELS
      AUTONOMY_SWITCH
    """

    __slots__ = [
        '_cmd',
        '_data',
    ]

    _fields_and_field_types = {
        'cmd': 'uint8',
        'data': 'sequence<uint8>',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('uint8')),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.cmd = kwargs.get('cmd', int())
        self.data = array.array('B', kwargs.get('data', []))

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
        if self.cmd != other.cmd:
            return False
        if self.data != other.data:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def cmd(self):
        """Message field 'cmd'."""
        return self._cmd

    @cmd.setter
    def cmd(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'cmd' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'cmd' field must be an unsigned integer in [0, 255]"
        self._cmd = value

    @builtins.property
    def data(self):
        """Message field 'data'."""
        return self._data

    @data.setter
    def data(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'B', \
                "The 'data' array.array() must have the type code of 'B'"
            self._data = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, int) for v in value) and
                 all(val >= 0 and val < 256 for val in value)), \
                "The 'data' field must be a set or sequence and each value of type 'int' and each unsigned integer in [0, 255]"
        self._data = array.array('B', value)

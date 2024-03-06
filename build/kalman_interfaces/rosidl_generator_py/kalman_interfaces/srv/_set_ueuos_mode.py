# generated from rosidl_generator_py/resource/_idl.py.em
# with input from kalman_interfaces:srv/SetUeuosMode.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_SetUeuosMode_Request(type):
    """Metaclass of message 'SetUeuosMode_Request'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'OFF': 0,
        'AUTONOMY': 1,
        'TELEOP': 2,
        'FINISHED': 3,
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
                'kalman_interfaces.srv.SetUeuosMode_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__set_ueuos_mode__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__set_ueuos_mode__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__set_ueuos_mode__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__set_ueuos_mode__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__set_ueuos_mode__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'OFF': cls.__constants['OFF'],
            'AUTONOMY': cls.__constants['AUTONOMY'],
            'TELEOP': cls.__constants['TELEOP'],
            'FINISHED': cls.__constants['FINISHED'],
        }

    @property
    def OFF(self):
        """Message constant 'OFF'."""
        return Metaclass_SetUeuosMode_Request.__constants['OFF']

    @property
    def AUTONOMY(self):
        """Message constant 'AUTONOMY'."""
        return Metaclass_SetUeuosMode_Request.__constants['AUTONOMY']

    @property
    def TELEOP(self):
        """Message constant 'TELEOP'."""
        return Metaclass_SetUeuosMode_Request.__constants['TELEOP']

    @property
    def FINISHED(self):
        """Message constant 'FINISHED'."""
        return Metaclass_SetUeuosMode_Request.__constants['FINISHED']


class SetUeuosMode_Request(metaclass=Metaclass_SetUeuosMode_Request):
    """
    Message class 'SetUeuosMode_Request'.

    Constants:
      OFF
      AUTONOMY
      TELEOP
      FINISHED
    """

    __slots__ = [
        '_mode',
    ]

    _fields_and_field_types = {
        'mode': 'uint8',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.mode = kwargs.get('mode', int())

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
        if self.mode != other.mode:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def mode(self):
        """Message field 'mode'."""
        return self._mode

    @mode.setter
    def mode(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'mode' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'mode' field must be an unsigned integer in [0, 255]"
        self._mode = value


# Import statements for member types

# already imported above
# import rosidl_parser.definition


class Metaclass_SetUeuosMode_Response(type):
    """Metaclass of message 'SetUeuosMode_Response'."""

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
                'kalman_interfaces.srv.SetUeuosMode_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__set_ueuos_mode__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__set_ueuos_mode__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__set_ueuos_mode__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__set_ueuos_mode__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__set_ueuos_mode__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class SetUeuosMode_Response(metaclass=Metaclass_SetUeuosMode_Response):
    """Message class 'SetUeuosMode_Response'."""

    __slots__ = [
    ]

    _fields_and_field_types = {
    }

    SLOT_TYPES = (
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))

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
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)


class Metaclass_SetUeuosMode(type):
    """Metaclass of service 'SetUeuosMode'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('kalman_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'kalman_interfaces.srv.SetUeuosMode')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__set_ueuos_mode

            from kalman_interfaces.srv import _set_ueuos_mode
            if _set_ueuos_mode.Metaclass_SetUeuosMode_Request._TYPE_SUPPORT is None:
                _set_ueuos_mode.Metaclass_SetUeuosMode_Request.__import_type_support__()
            if _set_ueuos_mode.Metaclass_SetUeuosMode_Response._TYPE_SUPPORT is None:
                _set_ueuos_mode.Metaclass_SetUeuosMode_Response.__import_type_support__()


class SetUeuosMode(metaclass=Metaclass_SetUeuosMode):
    from kalman_interfaces.srv._set_ueuos_mode import SetUeuosMode_Request as Request
    from kalman_interfaces.srv._set_ueuos_mode import SetUeuosMode_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')

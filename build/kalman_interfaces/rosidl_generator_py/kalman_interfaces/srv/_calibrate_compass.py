# generated from rosidl_generator_py/resource/_idl.py.em
# with input from kalman_interfaces:srv/CalibrateCompass.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_CalibrateCompass_Request(type):
    """Metaclass of message 'CalibrateCompass_Request'."""

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
                'kalman_interfaces.srv.CalibrateCompass_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__calibrate_compass__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__calibrate_compass__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__calibrate_compass__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__calibrate_compass__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__calibrate_compass__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'DURATION__DEFAULT': 30.0,
            'ANGULAR_VELOCITY__DEFAULT': 0.5,
        }

    @property
    def DURATION__DEFAULT(cls):
        """Return default value for message field 'duration'."""
        return 30.0

    @property
    def ANGULAR_VELOCITY__DEFAULT(cls):
        """Return default value for message field 'angular_velocity'."""
        return 0.5


class CalibrateCompass_Request(metaclass=Metaclass_CalibrateCompass_Request):
    """Message class 'CalibrateCompass_Request'."""

    __slots__ = [
        '_duration',
        '_angular_velocity',
    ]

    _fields_and_field_types = {
        'duration': 'float',
        'angular_velocity': 'float',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.duration = kwargs.get(
            'duration', CalibrateCompass_Request.DURATION__DEFAULT)
        self.angular_velocity = kwargs.get(
            'angular_velocity', CalibrateCompass_Request.ANGULAR_VELOCITY__DEFAULT)

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
        if self.duration != other.duration:
            return False
        if self.angular_velocity != other.angular_velocity:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def duration(self):
        """Message field 'duration'."""
        return self._duration

    @duration.setter
    def duration(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'duration' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'duration' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._duration = value

    @builtins.property
    def angular_velocity(self):
        """Message field 'angular_velocity'."""
        return self._angular_velocity

    @angular_velocity.setter
    def angular_velocity(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'angular_velocity' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'angular_velocity' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._angular_velocity = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import math

# already imported above
# import rosidl_parser.definition


class Metaclass_CalibrateCompass_Response(type):
    """Metaclass of message 'CalibrateCompass_Response'."""

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
                'kalman_interfaces.srv.CalibrateCompass_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__calibrate_compass__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__calibrate_compass__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__calibrate_compass__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__calibrate_compass__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__calibrate_compass__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class CalibrateCompass_Response(metaclass=Metaclass_CalibrateCompass_Response):
    """Message class 'CalibrateCompass_Response'."""

    __slots__ = [
        '_success',
        '_cc_mag_field',
        '_cc_offset0',
        '_cc_offset1',
        '_cc_offset2',
        '_cc_gain0',
        '_cc_gain1',
        '_cc_gain2',
        '_cc_t0',
        '_cc_t1',
        '_cc_t2',
        '_cc_t3',
        '_cc_t4',
        '_cc_t5',
    ]

    _fields_and_field_types = {
        'success': 'boolean',
        'cc_mag_field': 'float',
        'cc_offset0': 'float',
        'cc_offset1': 'float',
        'cc_offset2': 'float',
        'cc_gain0': 'float',
        'cc_gain1': 'float',
        'cc_gain2': 'float',
        'cc_t0': 'float',
        'cc_t1': 'float',
        'cc_t2': 'float',
        'cc_t3': 'float',
        'cc_t4': 'float',
        'cc_t5': 'float',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
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
        self.success = kwargs.get('success', bool())
        self.cc_mag_field = kwargs.get('cc_mag_field', float())
        self.cc_offset0 = kwargs.get('cc_offset0', float())
        self.cc_offset1 = kwargs.get('cc_offset1', float())
        self.cc_offset2 = kwargs.get('cc_offset2', float())
        self.cc_gain0 = kwargs.get('cc_gain0', float())
        self.cc_gain1 = kwargs.get('cc_gain1', float())
        self.cc_gain2 = kwargs.get('cc_gain2', float())
        self.cc_t0 = kwargs.get('cc_t0', float())
        self.cc_t1 = kwargs.get('cc_t1', float())
        self.cc_t2 = kwargs.get('cc_t2', float())
        self.cc_t3 = kwargs.get('cc_t3', float())
        self.cc_t4 = kwargs.get('cc_t4', float())
        self.cc_t5 = kwargs.get('cc_t5', float())

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
        if self.success != other.success:
            return False
        if self.cc_mag_field != other.cc_mag_field:
            return False
        if self.cc_offset0 != other.cc_offset0:
            return False
        if self.cc_offset1 != other.cc_offset1:
            return False
        if self.cc_offset2 != other.cc_offset2:
            return False
        if self.cc_gain0 != other.cc_gain0:
            return False
        if self.cc_gain1 != other.cc_gain1:
            return False
        if self.cc_gain2 != other.cc_gain2:
            return False
        if self.cc_t0 != other.cc_t0:
            return False
        if self.cc_t1 != other.cc_t1:
            return False
        if self.cc_t2 != other.cc_t2:
            return False
        if self.cc_t3 != other.cc_t3:
            return False
        if self.cc_t4 != other.cc_t4:
            return False
        if self.cc_t5 != other.cc_t5:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def success(self):
        """Message field 'success'."""
        return self._success

    @success.setter
    def success(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'success' field must be of type 'bool'"
        self._success = value

    @builtins.property
    def cc_mag_field(self):
        """Message field 'cc_mag_field'."""
        return self._cc_mag_field

    @cc_mag_field.setter
    def cc_mag_field(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'cc_mag_field' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'cc_mag_field' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._cc_mag_field = value

    @builtins.property
    def cc_offset0(self):
        """Message field 'cc_offset0'."""
        return self._cc_offset0

    @cc_offset0.setter
    def cc_offset0(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'cc_offset0' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'cc_offset0' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._cc_offset0 = value

    @builtins.property
    def cc_offset1(self):
        """Message field 'cc_offset1'."""
        return self._cc_offset1

    @cc_offset1.setter
    def cc_offset1(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'cc_offset1' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'cc_offset1' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._cc_offset1 = value

    @builtins.property
    def cc_offset2(self):
        """Message field 'cc_offset2'."""
        return self._cc_offset2

    @cc_offset2.setter
    def cc_offset2(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'cc_offset2' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'cc_offset2' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._cc_offset2 = value

    @builtins.property
    def cc_gain0(self):
        """Message field 'cc_gain0'."""
        return self._cc_gain0

    @cc_gain0.setter
    def cc_gain0(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'cc_gain0' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'cc_gain0' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._cc_gain0 = value

    @builtins.property
    def cc_gain1(self):
        """Message field 'cc_gain1'."""
        return self._cc_gain1

    @cc_gain1.setter
    def cc_gain1(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'cc_gain1' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'cc_gain1' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._cc_gain1 = value

    @builtins.property
    def cc_gain2(self):
        """Message field 'cc_gain2'."""
        return self._cc_gain2

    @cc_gain2.setter
    def cc_gain2(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'cc_gain2' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'cc_gain2' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._cc_gain2 = value

    @builtins.property
    def cc_t0(self):
        """Message field 'cc_t0'."""
        return self._cc_t0

    @cc_t0.setter
    def cc_t0(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'cc_t0' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'cc_t0' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._cc_t0 = value

    @builtins.property
    def cc_t1(self):
        """Message field 'cc_t1'."""
        return self._cc_t1

    @cc_t1.setter
    def cc_t1(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'cc_t1' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'cc_t1' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._cc_t1 = value

    @builtins.property
    def cc_t2(self):
        """Message field 'cc_t2'."""
        return self._cc_t2

    @cc_t2.setter
    def cc_t2(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'cc_t2' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'cc_t2' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._cc_t2 = value

    @builtins.property
    def cc_t3(self):
        """Message field 'cc_t3'."""
        return self._cc_t3

    @cc_t3.setter
    def cc_t3(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'cc_t3' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'cc_t3' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._cc_t3 = value

    @builtins.property
    def cc_t4(self):
        """Message field 'cc_t4'."""
        return self._cc_t4

    @cc_t4.setter
    def cc_t4(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'cc_t4' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'cc_t4' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._cc_t4 = value

    @builtins.property
    def cc_t5(self):
        """Message field 'cc_t5'."""
        return self._cc_t5

    @cc_t5.setter
    def cc_t5(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'cc_t5' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'cc_t5' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._cc_t5 = value


class Metaclass_CalibrateCompass(type):
    """Metaclass of service 'CalibrateCompass'."""

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
                'kalman_interfaces.srv.CalibrateCompass')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__calibrate_compass

            from kalman_interfaces.srv import _calibrate_compass
            if _calibrate_compass.Metaclass_CalibrateCompass_Request._TYPE_SUPPORT is None:
                _calibrate_compass.Metaclass_CalibrateCompass_Request.__import_type_support__()
            if _calibrate_compass.Metaclass_CalibrateCompass_Response._TYPE_SUPPORT is None:
                _calibrate_compass.Metaclass_CalibrateCompass_Response.__import_type_support__()


class CalibrateCompass(metaclass=Metaclass_CalibrateCompass):
    from kalman_interfaces.srv._calibrate_compass import CalibrateCompass_Request as Request
    from kalman_interfaces.srv._calibrate_compass import CalibrateCompass_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')

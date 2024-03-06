# generated from rosidl_generator_py/resource/_idl.py.em
# with input from service_based_nav2_controller_srvs:srv/ComputeVelocityCommands.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_ComputeVelocityCommands_Request(type):
    """Metaclass of message 'ComputeVelocityCommands_Request'."""

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
            module = import_type_support('service_based_nav2_controller_srvs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'service_based_nav2_controller_srvs.srv.ComputeVelocityCommands_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__compute_velocity_commands__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__compute_velocity_commands__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__compute_velocity_commands__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__compute_velocity_commands__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__compute_velocity_commands__request

            from geometry_msgs.msg import PoseStamped
            if PoseStamped.__class__._TYPE_SUPPORT is None:
                PoseStamped.__class__.__import_type_support__()

            from geometry_msgs.msg import Twist
            if Twist.__class__._TYPE_SUPPORT is None:
                Twist.__class__.__import_type_support__()

            from nav_msgs.msg import Path
            if Path.__class__._TYPE_SUPPORT is None:
                Path.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class ComputeVelocityCommands_Request(metaclass=Metaclass_ComputeVelocityCommands_Request):
    """Message class 'ComputeVelocityCommands_Request'."""

    __slots__ = [
        '_pose',
        '_velocity',
        '_path',
    ]

    _fields_and_field_types = {
        'pose': 'geometry_msgs/PoseStamped',
        'velocity': 'geometry_msgs/Twist',
        'path': 'nav_msgs/Path',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'PoseStamped'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Twist'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['nav_msgs', 'msg'], 'Path'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from geometry_msgs.msg import PoseStamped
        self.pose = kwargs.get('pose', PoseStamped())
        from geometry_msgs.msg import Twist
        self.velocity = kwargs.get('velocity', Twist())
        from nav_msgs.msg import Path
        self.path = kwargs.get('path', Path())

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
        if self.pose != other.pose:
            return False
        if self.velocity != other.velocity:
            return False
        if self.path != other.path:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def pose(self):
        """Message field 'pose'."""
        return self._pose

    @pose.setter
    def pose(self, value):
        if __debug__:
            from geometry_msgs.msg import PoseStamped
            assert \
                isinstance(value, PoseStamped), \
                "The 'pose' field must be a sub message of type 'PoseStamped'"
        self._pose = value

    @builtins.property
    def velocity(self):
        """Message field 'velocity'."""
        return self._velocity

    @velocity.setter
    def velocity(self, value):
        if __debug__:
            from geometry_msgs.msg import Twist
            assert \
                isinstance(value, Twist), \
                "The 'velocity' field must be a sub message of type 'Twist'"
        self._velocity = value

    @builtins.property
    def path(self):
        """Message field 'path'."""
        return self._path

    @path.setter
    def path(self, value):
        if __debug__:
            from nav_msgs.msg import Path
            assert \
                isinstance(value, Path), \
                "The 'path' field must be a sub message of type 'Path'"
        self._path = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_ComputeVelocityCommands_Response(type):
    """Metaclass of message 'ComputeVelocityCommands_Response'."""

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
            module = import_type_support('service_based_nav2_controller_srvs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'service_based_nav2_controller_srvs.srv.ComputeVelocityCommands_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__compute_velocity_commands__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__compute_velocity_commands__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__compute_velocity_commands__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__compute_velocity_commands__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__compute_velocity_commands__response

            from geometry_msgs.msg import TwistStamped
            if TwistStamped.__class__._TYPE_SUPPORT is None:
                TwistStamped.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class ComputeVelocityCommands_Response(metaclass=Metaclass_ComputeVelocityCommands_Response):
    """Message class 'ComputeVelocityCommands_Response'."""

    __slots__ = [
        '_cmd_vel',
    ]

    _fields_and_field_types = {
        'cmd_vel': 'geometry_msgs/TwistStamped',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'TwistStamped'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from geometry_msgs.msg import TwistStamped
        self.cmd_vel = kwargs.get('cmd_vel', TwistStamped())

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
        if self.cmd_vel != other.cmd_vel:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def cmd_vel(self):
        """Message field 'cmd_vel'."""
        return self._cmd_vel

    @cmd_vel.setter
    def cmd_vel(self, value):
        if __debug__:
            from geometry_msgs.msg import TwistStamped
            assert \
                isinstance(value, TwistStamped), \
                "The 'cmd_vel' field must be a sub message of type 'TwistStamped'"
        self._cmd_vel = value


class Metaclass_ComputeVelocityCommands(type):
    """Metaclass of service 'ComputeVelocityCommands'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('service_based_nav2_controller_srvs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'service_based_nav2_controller_srvs.srv.ComputeVelocityCommands')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__compute_velocity_commands

            from service_based_nav2_controller_srvs.srv import _compute_velocity_commands
            if _compute_velocity_commands.Metaclass_ComputeVelocityCommands_Request._TYPE_SUPPORT is None:
                _compute_velocity_commands.Metaclass_ComputeVelocityCommands_Request.__import_type_support__()
            if _compute_velocity_commands.Metaclass_ComputeVelocityCommands_Response._TYPE_SUPPORT is None:
                _compute_velocity_commands.Metaclass_ComputeVelocityCommands_Response.__import_type_support__()


class ComputeVelocityCommands(metaclass=Metaclass_ComputeVelocityCommands):
    from service_based_nav2_controller_srvs.srv._compute_velocity_commands import ComputeVelocityCommands_Request as Request
    from service_based_nav2_controller_srvs.srv._compute_velocity_commands import ComputeVelocityCommands_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')

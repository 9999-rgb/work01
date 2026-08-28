# generated from rosidl_generator_py/resource/_idl.py.em
# with input from xczs_inspection_robot_control:srv/SwitchToolset.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_SwitchToolset_Request(type):
    """Metaclass of message 'SwitchToolset_Request'."""

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
            module = import_type_support('xczs_inspection_robot_control')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'xczs_inspection_robot_control.srv.SwitchToolset_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__switch_toolset__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__switch_toolset__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__switch_toolset__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__switch_toolset__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__switch_toolset__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class SwitchToolset_Request(metaclass=Metaclass_SwitchToolset_Request):
    """Message class 'SwitchToolset_Request'."""

    __slots__ = [
        '_toolset',
        '_expected_generation',
    ]

    _fields_and_field_types = {
        'toolset': 'string',
        'expected_generation': 'uint64',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('uint64'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.toolset = kwargs.get('toolset', str())
        self.expected_generation = kwargs.get('expected_generation', int())

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
        if self.toolset != other.toolset:
            return False
        if self.expected_generation != other.expected_generation:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def toolset(self):
        """Message field 'toolset'."""
        return self._toolset

    @toolset.setter
    def toolset(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'toolset' field must be of type 'str'"
        self._toolset = value

    @builtins.property
    def expected_generation(self):
        """Message field 'expected_generation'."""
        return self._expected_generation

    @expected_generation.setter
    def expected_generation(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'expected_generation' field must be of type 'int'"
            assert value >= 0 and value < 18446744073709551616, \
                "The 'expected_generation' field must be an unsigned integer in [0, 18446744073709551615]"
        self._expected_generation = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_SwitchToolset_Response(type):
    """Metaclass of message 'SwitchToolset_Response'."""

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
            module = import_type_support('xczs_inspection_robot_control')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'xczs_inspection_robot_control.srv.SwitchToolset_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__switch_toolset__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__switch_toolset__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__switch_toolset__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__switch_toolset__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__switch_toolset__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class SwitchToolset_Response(metaclass=Metaclass_SwitchToolset_Response):
    """Message class 'SwitchToolset_Response'."""

    __slots__ = [
        '_accepted',
        '_state',
        '_active_toolset',
        '_target_toolset',
        '_generation',
        '_message',
    ]

    _fields_and_field_types = {
        'accepted': 'boolean',
        'state': 'string',
        'active_toolset': 'string',
        'target_toolset': 'string',
        'generation': 'uint64',
        'message': 'string',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('uint64'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.accepted = kwargs.get('accepted', bool())
        self.state = kwargs.get('state', str())
        self.active_toolset = kwargs.get('active_toolset', str())
        self.target_toolset = kwargs.get('target_toolset', str())
        self.generation = kwargs.get('generation', int())
        self.message = kwargs.get('message', str())

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
        if self.accepted != other.accepted:
            return False
        if self.state != other.state:
            return False
        if self.active_toolset != other.active_toolset:
            return False
        if self.target_toolset != other.target_toolset:
            return False
        if self.generation != other.generation:
            return False
        if self.message != other.message:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def accepted(self):
        """Message field 'accepted'."""
        return self._accepted

    @accepted.setter
    def accepted(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'accepted' field must be of type 'bool'"
        self._accepted = value

    @builtins.property
    def state(self):
        """Message field 'state'."""
        return self._state

    @state.setter
    def state(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'state' field must be of type 'str'"
        self._state = value

    @builtins.property
    def active_toolset(self):
        """Message field 'active_toolset'."""
        return self._active_toolset

    @active_toolset.setter
    def active_toolset(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'active_toolset' field must be of type 'str'"
        self._active_toolset = value

    @builtins.property
    def target_toolset(self):
        """Message field 'target_toolset'."""
        return self._target_toolset

    @target_toolset.setter
    def target_toolset(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'target_toolset' field must be of type 'str'"
        self._target_toolset = value

    @builtins.property
    def generation(self):
        """Message field 'generation'."""
        return self._generation

    @generation.setter
    def generation(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'generation' field must be of type 'int'"
            assert value >= 0 and value < 18446744073709551616, \
                "The 'generation' field must be an unsigned integer in [0, 18446744073709551615]"
        self._generation = value

    @builtins.property
    def message(self):
        """Message field 'message'."""
        return self._message

    @message.setter
    def message(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'message' field must be of type 'str'"
        self._message = value


class Metaclass_SwitchToolset(type):
    """Metaclass of service 'SwitchToolset'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('xczs_inspection_robot_control')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'xczs_inspection_robot_control.srv.SwitchToolset')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__switch_toolset

            from xczs_inspection_robot_control.srv import _switch_toolset
            if _switch_toolset.Metaclass_SwitchToolset_Request._TYPE_SUPPORT is None:
                _switch_toolset.Metaclass_SwitchToolset_Request.__import_type_support__()
            if _switch_toolset.Metaclass_SwitchToolset_Response._TYPE_SUPPORT is None:
                _switch_toolset.Metaclass_SwitchToolset_Response.__import_type_support__()


class SwitchToolset(metaclass=Metaclass_SwitchToolset):
    from xczs_inspection_robot_control.srv._switch_toolset import SwitchToolset_Request as Request
    from xczs_inspection_robot_control.srv._switch_toolset import SwitchToolset_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')

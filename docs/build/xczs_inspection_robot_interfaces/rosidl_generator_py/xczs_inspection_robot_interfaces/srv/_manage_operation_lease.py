# generated from rosidl_generator_py/resource/_idl.py.em
# with input from xczs_inspection_robot_interfaces:srv/ManageOperationLease.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_ManageOperationLease_Request(type):
    """Metaclass of message 'ManageOperationLease_Request'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'ACQUIRE': 0,
        'RENEW': 1,
        'RELEASE': 2,
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('xczs_inspection_robot_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'xczs_inspection_robot_interfaces.srv.ManageOperationLease_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__manage_operation_lease__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__manage_operation_lease__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__manage_operation_lease__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__manage_operation_lease__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__manage_operation_lease__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'ACQUIRE': cls.__constants['ACQUIRE'],
            'RENEW': cls.__constants['RENEW'],
            'RELEASE': cls.__constants['RELEASE'],
        }

    @property
    def ACQUIRE(self):
        """Message constant 'ACQUIRE'."""
        return Metaclass_ManageOperationLease_Request.__constants['ACQUIRE']

    @property
    def RENEW(self):
        """Message constant 'RENEW'."""
        return Metaclass_ManageOperationLease_Request.__constants['RENEW']

    @property
    def RELEASE(self):
        """Message constant 'RELEASE'."""
        return Metaclass_ManageOperationLease_Request.__constants['RELEASE']


class ManageOperationLease_Request(metaclass=Metaclass_ManageOperationLease_Request):
    """
    Message class 'ManageOperationLease_Request'.

    Constants:
      ACQUIRE
      RENEW
      RELEASE
    """

    __slots__ = [
        '_command',
        '_owner_id',
        '_lease_id',
        '_requested_duration',
    ]

    _fields_and_field_types = {
        'command': 'uint8',
        'owner_id': 'string',
        'lease_id': 'string',
        'requested_duration': 'double',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.command = kwargs.get('command', int())
        self.owner_id = kwargs.get('owner_id', str())
        self.lease_id = kwargs.get('lease_id', str())
        self.requested_duration = kwargs.get('requested_duration', float())

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
        if self.command != other.command:
            return False
        if self.owner_id != other.owner_id:
            return False
        if self.lease_id != other.lease_id:
            return False
        if self.requested_duration != other.requested_duration:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def command(self):
        """Message field 'command'."""
        return self._command

    @command.setter
    def command(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'command' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'command' field must be an unsigned integer in [0, 255]"
        self._command = value

    @builtins.property
    def owner_id(self):
        """Message field 'owner_id'."""
        return self._owner_id

    @owner_id.setter
    def owner_id(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'owner_id' field must be of type 'str'"
        self._owner_id = value

    @builtins.property
    def lease_id(self):
        """Message field 'lease_id'."""
        return self._lease_id

    @lease_id.setter
    def lease_id(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'lease_id' field must be of type 'str'"
        self._lease_id = value

    @builtins.property
    def requested_duration(self):
        """Message field 'requested_duration'."""
        return self._requested_duration

    @requested_duration.setter
    def requested_duration(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'requested_duration' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'requested_duration' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._requested_duration = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import math

# already imported above
# import rosidl_parser.definition


class Metaclass_ManageOperationLease_Response(type):
    """Metaclass of message 'ManageOperationLease_Response'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'GRANTED': 0,
        'RESOURCE_BUSY': 1,
        'INVALID_REQUEST': 2,
        'NOT_OWNER': 3,
        'RELEASED': 4,
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('xczs_inspection_robot_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'xczs_inspection_robot_interfaces.srv.ManageOperationLease_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__manage_operation_lease__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__manage_operation_lease__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__manage_operation_lease__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__manage_operation_lease__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__manage_operation_lease__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'GRANTED': cls.__constants['GRANTED'],
            'RESOURCE_BUSY': cls.__constants['RESOURCE_BUSY'],
            'INVALID_REQUEST': cls.__constants['INVALID_REQUEST'],
            'NOT_OWNER': cls.__constants['NOT_OWNER'],
            'RELEASED': cls.__constants['RELEASED'],
        }

    @property
    def GRANTED(self):
        """Message constant 'GRANTED'."""
        return Metaclass_ManageOperationLease_Response.__constants['GRANTED']

    @property
    def RESOURCE_BUSY(self):
        """Message constant 'RESOURCE_BUSY'."""
        return Metaclass_ManageOperationLease_Response.__constants['RESOURCE_BUSY']

    @property
    def INVALID_REQUEST(self):
        """Message constant 'INVALID_REQUEST'."""
        return Metaclass_ManageOperationLease_Response.__constants['INVALID_REQUEST']

    @property
    def NOT_OWNER(self):
        """Message constant 'NOT_OWNER'."""
        return Metaclass_ManageOperationLease_Response.__constants['NOT_OWNER']

    @property
    def RELEASED(self):
        """Message constant 'RELEASED'."""
        return Metaclass_ManageOperationLease_Response.__constants['RELEASED']


class ManageOperationLease_Response(metaclass=Metaclass_ManageOperationLease_Response):
    """
    Message class 'ManageOperationLease_Response'.

    Constants:
      GRANTED
      RESOURCE_BUSY
      INVALID_REQUEST
      NOT_OWNER
      RELEASED
    """

    __slots__ = [
        '_success',
        '_status_code',
        '_message',
        '_lease_id',
        '_owner_id',
        '_remaining_duration',
    ]

    _fields_and_field_types = {
        'success': 'boolean',
        'status_code': 'uint8',
        'message': 'string',
        'lease_id': 'string',
        'owner_id': 'string',
        'remaining_duration': 'double',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.success = kwargs.get('success', bool())
        self.status_code = kwargs.get('status_code', int())
        self.message = kwargs.get('message', str())
        self.lease_id = kwargs.get('lease_id', str())
        self.owner_id = kwargs.get('owner_id', str())
        self.remaining_duration = kwargs.get('remaining_duration', float())

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
        if self.status_code != other.status_code:
            return False
        if self.message != other.message:
            return False
        if self.lease_id != other.lease_id:
            return False
        if self.owner_id != other.owner_id:
            return False
        if self.remaining_duration != other.remaining_duration:
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
    def status_code(self):
        """Message field 'status_code'."""
        return self._status_code

    @status_code.setter
    def status_code(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'status_code' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'status_code' field must be an unsigned integer in [0, 255]"
        self._status_code = value

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

    @builtins.property
    def lease_id(self):
        """Message field 'lease_id'."""
        return self._lease_id

    @lease_id.setter
    def lease_id(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'lease_id' field must be of type 'str'"
        self._lease_id = value

    @builtins.property
    def owner_id(self):
        """Message field 'owner_id'."""
        return self._owner_id

    @owner_id.setter
    def owner_id(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'owner_id' field must be of type 'str'"
        self._owner_id = value

    @builtins.property
    def remaining_duration(self):
        """Message field 'remaining_duration'."""
        return self._remaining_duration

    @remaining_duration.setter
    def remaining_duration(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'remaining_duration' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'remaining_duration' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._remaining_duration = value


class Metaclass_ManageOperationLease(type):
    """Metaclass of service 'ManageOperationLease'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('xczs_inspection_robot_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'xczs_inspection_robot_interfaces.srv.ManageOperationLease')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__manage_operation_lease

            from xczs_inspection_robot_interfaces.srv import _manage_operation_lease
            if _manage_operation_lease.Metaclass_ManageOperationLease_Request._TYPE_SUPPORT is None:
                _manage_operation_lease.Metaclass_ManageOperationLease_Request.__import_type_support__()
            if _manage_operation_lease.Metaclass_ManageOperationLease_Response._TYPE_SUPPORT is None:
                _manage_operation_lease.Metaclass_ManageOperationLease_Response.__import_type_support__()


class ManageOperationLease(metaclass=Metaclass_ManageOperationLease):
    from xczs_inspection_robot_interfaces.srv._manage_operation_lease import ManageOperationLease_Request as Request
    from xczs_inspection_robot_interfaces.srv._manage_operation_lease import ManageOperationLease_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')

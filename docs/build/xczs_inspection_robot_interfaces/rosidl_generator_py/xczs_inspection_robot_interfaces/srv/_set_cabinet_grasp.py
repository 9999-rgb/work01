# generated from rosidl_generator_py/resource/_idl.py.em
# with input from xczs_inspection_robot_interfaces:srv/SetCabinetGrasp.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_SetCabinetGrasp_Request(type):
    """Metaclass of message 'SetCabinetGrasp_Request'."""

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
            module = import_type_support('xczs_inspection_robot_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'xczs_inspection_robot_interfaces.srv.SetCabinetGrasp_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__set_cabinet_grasp__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__set_cabinet_grasp__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__set_cabinet_grasp__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__set_cabinet_grasp__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__set_cabinet_grasp__request

            from geometry_msgs.msg import Point
            if Point.__class__._TYPE_SUPPORT is None:
                Point.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class SetCabinetGrasp_Request(metaclass=Metaclass_SetCabinetGrasp_Request):
    """Message class 'SetCabinetGrasp_Request'."""

    __slots__ = [
        '_control_id',
        '_operation_lease_id',
        '_robot_model',
        '_robot_link',
        '_robot_grasp_point',
        '_robot_base_link',
        '_attach',
    ]

    _fields_and_field_types = {
        'control_id': 'string',
        'operation_lease_id': 'string',
        'robot_model': 'string',
        'robot_link': 'string',
        'robot_grasp_point': 'geometry_msgs/Point',
        'robot_base_link': 'string',
        'attach': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Point'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.control_id = kwargs.get('control_id', str())
        self.operation_lease_id = kwargs.get('operation_lease_id', str())
        self.robot_model = kwargs.get('robot_model', str())
        self.robot_link = kwargs.get('robot_link', str())
        from geometry_msgs.msg import Point
        self.robot_grasp_point = kwargs.get('robot_grasp_point', Point())
        self.robot_base_link = kwargs.get('robot_base_link', str())
        self.attach = kwargs.get('attach', bool())

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
        if self.control_id != other.control_id:
            return False
        if self.operation_lease_id != other.operation_lease_id:
            return False
        if self.robot_model != other.robot_model:
            return False
        if self.robot_link != other.robot_link:
            return False
        if self.robot_grasp_point != other.robot_grasp_point:
            return False
        if self.robot_base_link != other.robot_base_link:
            return False
        if self.attach != other.attach:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def control_id(self):
        """Message field 'control_id'."""
        return self._control_id

    @control_id.setter
    def control_id(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'control_id' field must be of type 'str'"
        self._control_id = value

    @builtins.property
    def operation_lease_id(self):
        """Message field 'operation_lease_id'."""
        return self._operation_lease_id

    @operation_lease_id.setter
    def operation_lease_id(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'operation_lease_id' field must be of type 'str'"
        self._operation_lease_id = value

    @builtins.property
    def robot_model(self):
        """Message field 'robot_model'."""
        return self._robot_model

    @robot_model.setter
    def robot_model(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'robot_model' field must be of type 'str'"
        self._robot_model = value

    @builtins.property
    def robot_link(self):
        """Message field 'robot_link'."""
        return self._robot_link

    @robot_link.setter
    def robot_link(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'robot_link' field must be of type 'str'"
        self._robot_link = value

    @builtins.property
    def robot_grasp_point(self):
        """Message field 'robot_grasp_point'."""
        return self._robot_grasp_point

    @robot_grasp_point.setter
    def robot_grasp_point(self, value):
        if __debug__:
            from geometry_msgs.msg import Point
            assert \
                isinstance(value, Point), \
                "The 'robot_grasp_point' field must be a sub message of type 'Point'"
        self._robot_grasp_point = value

    @builtins.property
    def robot_base_link(self):
        """Message field 'robot_base_link'."""
        return self._robot_base_link

    @robot_base_link.setter
    def robot_base_link(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'robot_base_link' field must be of type 'str'"
        self._robot_base_link = value

    @builtins.property
    def attach(self):
        """Message field 'attach'."""
        return self._attach

    @attach.setter
    def attach(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'attach' field must be of type 'bool'"
        self._attach = value


# Import statements for member types

# already imported above
# import builtins

import math  # noqa: E402, I100

# already imported above
# import rosidl_parser.definition


class Metaclass_SetCabinetGrasp_Response(type):
    """Metaclass of message 'SetCabinetGrasp_Response'."""

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
            module = import_type_support('xczs_inspection_robot_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'xczs_inspection_robot_interfaces.srv.SetCabinetGrasp_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__set_cabinet_grasp__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__set_cabinet_grasp__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__set_cabinet_grasp__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__set_cabinet_grasp__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__set_cabinet_grasp__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class SetCabinetGrasp_Response(metaclass=Metaclass_SetCabinetGrasp_Response):
    """Message class 'SetCabinetGrasp_Response'."""

    __slots__ = [
        '_success',
        '_message',
        '_distance',
    ]

    _fields_and_field_types = {
        'success': 'boolean',
        'message': 'string',
        'distance': 'double',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.success = kwargs.get('success', bool())
        self.message = kwargs.get('message', str())
        self.distance = kwargs.get('distance', float())

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
        if self.message != other.message:
            return False
        if self.distance != other.distance:
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
    def distance(self):
        """Message field 'distance'."""
        return self._distance

    @distance.setter
    def distance(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'distance' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'distance' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._distance = value


class Metaclass_SetCabinetGrasp(type):
    """Metaclass of service 'SetCabinetGrasp'."""

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
                'xczs_inspection_robot_interfaces.srv.SetCabinetGrasp')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__set_cabinet_grasp

            from xczs_inspection_robot_interfaces.srv import _set_cabinet_grasp
            if _set_cabinet_grasp.Metaclass_SetCabinetGrasp_Request._TYPE_SUPPORT is None:
                _set_cabinet_grasp.Metaclass_SetCabinetGrasp_Request.__import_type_support__()
            if _set_cabinet_grasp.Metaclass_SetCabinetGrasp_Response._TYPE_SUPPORT is None:
                _set_cabinet_grasp.Metaclass_SetCabinetGrasp_Response.__import_type_support__()


class SetCabinetGrasp(metaclass=Metaclass_SetCabinetGrasp):
    from xczs_inspection_robot_interfaces.srv._set_cabinet_grasp import SetCabinetGrasp_Request as Request
    from xczs_inspection_robot_interfaces.srv._set_cabinet_grasp import SetCabinetGrasp_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')

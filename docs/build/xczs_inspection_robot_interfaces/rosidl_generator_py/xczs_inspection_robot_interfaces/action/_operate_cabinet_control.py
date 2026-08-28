# generated from rosidl_generator_py/resource/_idl.py.em
# with input from xczs_inspection_robot_interfaces:action/OperateCabinetControl.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_OperateCabinetControl_Goal(type):
    """Metaclass of message 'OperateCabinetControl_Goal'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'COMMAND_PRESS': 0,
        'COMMAND_SET_STATE': 1,
        'COMMAND_SET_POSITION': 2,
        'COMMAND_TOGGLE': 3,
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
                'xczs_inspection_robot_interfaces.action.OperateCabinetControl_Goal')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__operate_cabinet_control__goal
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__operate_cabinet_control__goal
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__operate_cabinet_control__goal
            cls._TYPE_SUPPORT = module.type_support_msg__action__operate_cabinet_control__goal
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__operate_cabinet_control__goal

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'COMMAND_PRESS': cls.__constants['COMMAND_PRESS'],
            'COMMAND_SET_STATE': cls.__constants['COMMAND_SET_STATE'],
            'COMMAND_SET_POSITION': cls.__constants['COMMAND_SET_POSITION'],
            'COMMAND_TOGGLE': cls.__constants['COMMAND_TOGGLE'],
        }

    @property
    def COMMAND_PRESS(self):
        """Message constant 'COMMAND_PRESS'."""
        return Metaclass_OperateCabinetControl_Goal.__constants['COMMAND_PRESS']

    @property
    def COMMAND_SET_STATE(self):
        """Message constant 'COMMAND_SET_STATE'."""
        return Metaclass_OperateCabinetControl_Goal.__constants['COMMAND_SET_STATE']

    @property
    def COMMAND_SET_POSITION(self):
        """Message constant 'COMMAND_SET_POSITION'."""
        return Metaclass_OperateCabinetControl_Goal.__constants['COMMAND_SET_POSITION']

    @property
    def COMMAND_TOGGLE(self):
        """Message constant 'COMMAND_TOGGLE'."""
        return Metaclass_OperateCabinetControl_Goal.__constants['COMMAND_TOGGLE']


class OperateCabinetControl_Goal(metaclass=Metaclass_OperateCabinetControl_Goal):
    """
    Message class 'OperateCabinetControl_Goal'.

    Constants:
      COMMAND_PRESS
      COMMAND_SET_STATE
      COMMAND_SET_POSITION
      COMMAND_TOGGLE
    """

    __slots__ = [
        '_control_id',
        '_command',
        '_target_state',
        '_target_position',
        '_use_target_position',
        '_force',
        '_navigate_to_staging_pose',
    ]

    _fields_and_field_types = {
        'control_id': 'string',
        'command': 'uint8',
        'target_state': 'string',
        'target_position': 'double',
        'use_target_position': 'boolean',
        'force': 'double',
        'navigate_to_staging_pose': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.control_id = kwargs.get('control_id', str())
        self.command = kwargs.get('command', int())
        self.target_state = kwargs.get('target_state', str())
        self.target_position = kwargs.get('target_position', float())
        self.use_target_position = kwargs.get('use_target_position', bool())
        self.force = kwargs.get('force', float())
        self.navigate_to_staging_pose = kwargs.get('navigate_to_staging_pose', bool())

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
        if self.command != other.command:
            return False
        if self.target_state != other.target_state:
            return False
        if self.target_position != other.target_position:
            return False
        if self.use_target_position != other.use_target_position:
            return False
        if self.force != other.force:
            return False
        if self.navigate_to_staging_pose != other.navigate_to_staging_pose:
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
    def target_state(self):
        """Message field 'target_state'."""
        return self._target_state

    @target_state.setter
    def target_state(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'target_state' field must be of type 'str'"
        self._target_state = value

    @builtins.property
    def target_position(self):
        """Message field 'target_position'."""
        return self._target_position

    @target_position.setter
    def target_position(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'target_position' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'target_position' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._target_position = value

    @builtins.property
    def use_target_position(self):
        """Message field 'use_target_position'."""
        return self._use_target_position

    @use_target_position.setter
    def use_target_position(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'use_target_position' field must be of type 'bool'"
        self._use_target_position = value

    @builtins.property
    def force(self):
        """Message field 'force'."""
        return self._force

    @force.setter
    def force(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'force' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'force' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._force = value

    @builtins.property
    def navigate_to_staging_pose(self):
        """Message field 'navigate_to_staging_pose'."""
        return self._navigate_to_staging_pose

    @navigate_to_staging_pose.setter
    def navigate_to_staging_pose(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'navigate_to_staging_pose' field must be of type 'bool'"
        self._navigate_to_staging_pose = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import math

# already imported above
# import rosidl_parser.definition


class Metaclass_OperateCabinetControl_Result(type):
    """Metaclass of message 'OperateCabinetControl_Result'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'SUCCESS': 0,
        'INVALID_CONTROL': 1,
        'UNSUPPORTED_COMMAND': 2,
        'NOT_READY': 3,
        'NAVIGATION_FAILED': 4,
        'PLANNING_FAILED': 5,
        'EXECUTION_FAILED': 6,
        'GRASP_FAILED': 7,
        'TARGET_NOT_REACHED': 8,
        'RELEASE_FAILED': 9,
        'CANCELED': 10,
        'INTERNAL_ERROR': 11,
        'INVALID_FORCE': 12,
        'INSUFFICIENT_FORCE': 13,
        'UNREACHABLE': 14,
        'CONTACT_DETECTION_TIMEOUT': 15,
        'RESOURCE_BUSY': 16,
        'LEASE_LOST': 17,
        'TOOLSET_MISMATCH': 18,
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
                'xczs_inspection_robot_interfaces.action.OperateCabinetControl_Result')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__operate_cabinet_control__result
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__operate_cabinet_control__result
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__operate_cabinet_control__result
            cls._TYPE_SUPPORT = module.type_support_msg__action__operate_cabinet_control__result
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__operate_cabinet_control__result

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'SUCCESS': cls.__constants['SUCCESS'],
            'INVALID_CONTROL': cls.__constants['INVALID_CONTROL'],
            'UNSUPPORTED_COMMAND': cls.__constants['UNSUPPORTED_COMMAND'],
            'NOT_READY': cls.__constants['NOT_READY'],
            'NAVIGATION_FAILED': cls.__constants['NAVIGATION_FAILED'],
            'PLANNING_FAILED': cls.__constants['PLANNING_FAILED'],
            'EXECUTION_FAILED': cls.__constants['EXECUTION_FAILED'],
            'GRASP_FAILED': cls.__constants['GRASP_FAILED'],
            'TARGET_NOT_REACHED': cls.__constants['TARGET_NOT_REACHED'],
            'RELEASE_FAILED': cls.__constants['RELEASE_FAILED'],
            'CANCELED': cls.__constants['CANCELED'],
            'INTERNAL_ERROR': cls.__constants['INTERNAL_ERROR'],
            'INVALID_FORCE': cls.__constants['INVALID_FORCE'],
            'INSUFFICIENT_FORCE': cls.__constants['INSUFFICIENT_FORCE'],
            'UNREACHABLE': cls.__constants['UNREACHABLE'],
            'CONTACT_DETECTION_TIMEOUT': cls.__constants['CONTACT_DETECTION_TIMEOUT'],
            'RESOURCE_BUSY': cls.__constants['RESOURCE_BUSY'],
            'LEASE_LOST': cls.__constants['LEASE_LOST'],
            'TOOLSET_MISMATCH': cls.__constants['TOOLSET_MISMATCH'],
        }

    @property
    def SUCCESS(self):
        """Message constant 'SUCCESS'."""
        return Metaclass_OperateCabinetControl_Result.__constants['SUCCESS']

    @property
    def INVALID_CONTROL(self):
        """Message constant 'INVALID_CONTROL'."""
        return Metaclass_OperateCabinetControl_Result.__constants['INVALID_CONTROL']

    @property
    def UNSUPPORTED_COMMAND(self):
        """Message constant 'UNSUPPORTED_COMMAND'."""
        return Metaclass_OperateCabinetControl_Result.__constants['UNSUPPORTED_COMMAND']

    @property
    def NOT_READY(self):
        """Message constant 'NOT_READY'."""
        return Metaclass_OperateCabinetControl_Result.__constants['NOT_READY']

    @property
    def NAVIGATION_FAILED(self):
        """Message constant 'NAVIGATION_FAILED'."""
        return Metaclass_OperateCabinetControl_Result.__constants['NAVIGATION_FAILED']

    @property
    def PLANNING_FAILED(self):
        """Message constant 'PLANNING_FAILED'."""
        return Metaclass_OperateCabinetControl_Result.__constants['PLANNING_FAILED']

    @property
    def EXECUTION_FAILED(self):
        """Message constant 'EXECUTION_FAILED'."""
        return Metaclass_OperateCabinetControl_Result.__constants['EXECUTION_FAILED']

    @property
    def GRASP_FAILED(self):
        """Message constant 'GRASP_FAILED'."""
        return Metaclass_OperateCabinetControl_Result.__constants['GRASP_FAILED']

    @property
    def TARGET_NOT_REACHED(self):
        """Message constant 'TARGET_NOT_REACHED'."""
        return Metaclass_OperateCabinetControl_Result.__constants['TARGET_NOT_REACHED']

    @property
    def RELEASE_FAILED(self):
        """Message constant 'RELEASE_FAILED'."""
        return Metaclass_OperateCabinetControl_Result.__constants['RELEASE_FAILED']

    @property
    def CANCELED(self):
        """Message constant 'CANCELED'."""
        return Metaclass_OperateCabinetControl_Result.__constants['CANCELED']

    @property
    def INTERNAL_ERROR(self):
        """Message constant 'INTERNAL_ERROR'."""
        return Metaclass_OperateCabinetControl_Result.__constants['INTERNAL_ERROR']

    @property
    def INVALID_FORCE(self):
        """Message constant 'INVALID_FORCE'."""
        return Metaclass_OperateCabinetControl_Result.__constants['INVALID_FORCE']

    @property
    def INSUFFICIENT_FORCE(self):
        """Message constant 'INSUFFICIENT_FORCE'."""
        return Metaclass_OperateCabinetControl_Result.__constants['INSUFFICIENT_FORCE']

    @property
    def UNREACHABLE(self):
        """Message constant 'UNREACHABLE'."""
        return Metaclass_OperateCabinetControl_Result.__constants['UNREACHABLE']

    @property
    def CONTACT_DETECTION_TIMEOUT(self):
        """Message constant 'CONTACT_DETECTION_TIMEOUT'."""
        return Metaclass_OperateCabinetControl_Result.__constants['CONTACT_DETECTION_TIMEOUT']

    @property
    def RESOURCE_BUSY(self):
        """Message constant 'RESOURCE_BUSY'."""
        return Metaclass_OperateCabinetControl_Result.__constants['RESOURCE_BUSY']

    @property
    def LEASE_LOST(self):
        """Message constant 'LEASE_LOST'."""
        return Metaclass_OperateCabinetControl_Result.__constants['LEASE_LOST']

    @property
    def TOOLSET_MISMATCH(self):
        """Message constant 'TOOLSET_MISMATCH'."""
        return Metaclass_OperateCabinetControl_Result.__constants['TOOLSET_MISMATCH']


class OperateCabinetControl_Result(metaclass=Metaclass_OperateCabinetControl_Result):
    """
    Message class 'OperateCabinetControl_Result'.

    Constants:
      SUCCESS
      INVALID_CONTROL
      UNSUPPORTED_COMMAND
      NOT_READY
      NAVIGATION_FAILED
      PLANNING_FAILED
      EXECUTION_FAILED
      GRASP_FAILED
      TARGET_NOT_REACHED
      RELEASE_FAILED
      CANCELED
      INTERNAL_ERROR
      INVALID_FORCE
      INSUFFICIENT_FORCE
      UNREACHABLE
      CONTACT_DETECTION_TIMEOUT
      RESOURCE_BUSY
      LEASE_LOST
      TOOLSET_MISMATCH
    """

    __slots__ = [
        '_success',
        '_error_code',
        '_message',
        '_initial_position',
        '_final_position',
        '_peak_position',
        '_final_state',
        '_requested_force',
        '_estimated_force',
        '_button_triggered',
        '_validation_performed',
        '_operation_executed',
        '_diagnostic_stage',
        '_path_fraction',
        '_required_fraction',
        '_moveit_error_code',
        '_policy_reason',
        '_failure_reason',
        '_physical_outcome_confirmed',
        '_final_state_verified',
        '_transport_succeeded',
        '_recovery_succeeded',
        '_grasp_released',
    ]

    _fields_and_field_types = {
        'success': 'boolean',
        'error_code': 'uint8',
        'message': 'string',
        'initial_position': 'double',
        'final_position': 'double',
        'peak_position': 'double',
        'final_state': 'string',
        'requested_force': 'double',
        'estimated_force': 'double',
        'button_triggered': 'boolean',
        'validation_performed': 'boolean',
        'operation_executed': 'boolean',
        'diagnostic_stage': 'string',
        'path_fraction': 'double',
        'required_fraction': 'double',
        'moveit_error_code': 'int32',
        'policy_reason': 'string',
        'failure_reason': 'string',
        'physical_outcome_confirmed': 'boolean',
        'final_state_verified': 'boolean',
        'transport_succeeded': 'boolean',
        'recovery_succeeded': 'boolean',
        'grasp_released': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.success = kwargs.get('success', bool())
        self.error_code = kwargs.get('error_code', int())
        self.message = kwargs.get('message', str())
        self.initial_position = kwargs.get('initial_position', float())
        self.final_position = kwargs.get('final_position', float())
        self.peak_position = kwargs.get('peak_position', float())
        self.final_state = kwargs.get('final_state', str())
        self.requested_force = kwargs.get('requested_force', float())
        self.estimated_force = kwargs.get('estimated_force', float())
        self.button_triggered = kwargs.get('button_triggered', bool())
        self.validation_performed = kwargs.get('validation_performed', bool())
        self.operation_executed = kwargs.get('operation_executed', bool())
        self.diagnostic_stage = kwargs.get('diagnostic_stage', str())
        self.path_fraction = kwargs.get('path_fraction', float())
        self.required_fraction = kwargs.get('required_fraction', float())
        self.moveit_error_code = kwargs.get('moveit_error_code', int())
        self.policy_reason = kwargs.get('policy_reason', str())
        self.failure_reason = kwargs.get('failure_reason', str())
        self.physical_outcome_confirmed = kwargs.get('physical_outcome_confirmed', bool())
        self.final_state_verified = kwargs.get('final_state_verified', bool())
        self.transport_succeeded = kwargs.get('transport_succeeded', bool())
        self.recovery_succeeded = kwargs.get('recovery_succeeded', bool())
        self.grasp_released = kwargs.get('grasp_released', bool())

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
        if self.error_code != other.error_code:
            return False
        if self.message != other.message:
            return False
        if self.initial_position != other.initial_position:
            return False
        if self.final_position != other.final_position:
            return False
        if self.peak_position != other.peak_position:
            return False
        if self.final_state != other.final_state:
            return False
        if self.requested_force != other.requested_force:
            return False
        if self.estimated_force != other.estimated_force:
            return False
        if self.button_triggered != other.button_triggered:
            return False
        if self.validation_performed != other.validation_performed:
            return False
        if self.operation_executed != other.operation_executed:
            return False
        if self.diagnostic_stage != other.diagnostic_stage:
            return False
        if self.path_fraction != other.path_fraction:
            return False
        if self.required_fraction != other.required_fraction:
            return False
        if self.moveit_error_code != other.moveit_error_code:
            return False
        if self.policy_reason != other.policy_reason:
            return False
        if self.failure_reason != other.failure_reason:
            return False
        if self.physical_outcome_confirmed != other.physical_outcome_confirmed:
            return False
        if self.final_state_verified != other.final_state_verified:
            return False
        if self.transport_succeeded != other.transport_succeeded:
            return False
        if self.recovery_succeeded != other.recovery_succeeded:
            return False
        if self.grasp_released != other.grasp_released:
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
    def error_code(self):
        """Message field 'error_code'."""
        return self._error_code

    @error_code.setter
    def error_code(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'error_code' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'error_code' field must be an unsigned integer in [0, 255]"
        self._error_code = value

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
    def initial_position(self):
        """Message field 'initial_position'."""
        return self._initial_position

    @initial_position.setter
    def initial_position(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'initial_position' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'initial_position' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._initial_position = value

    @builtins.property
    def final_position(self):
        """Message field 'final_position'."""
        return self._final_position

    @final_position.setter
    def final_position(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'final_position' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'final_position' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._final_position = value

    @builtins.property
    def peak_position(self):
        """Message field 'peak_position'."""
        return self._peak_position

    @peak_position.setter
    def peak_position(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'peak_position' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'peak_position' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._peak_position = value

    @builtins.property
    def final_state(self):
        """Message field 'final_state'."""
        return self._final_state

    @final_state.setter
    def final_state(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'final_state' field must be of type 'str'"
        self._final_state = value

    @builtins.property
    def requested_force(self):
        """Message field 'requested_force'."""
        return self._requested_force

    @requested_force.setter
    def requested_force(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'requested_force' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'requested_force' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._requested_force = value

    @builtins.property
    def estimated_force(self):
        """Message field 'estimated_force'."""
        return self._estimated_force

    @estimated_force.setter
    def estimated_force(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'estimated_force' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'estimated_force' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._estimated_force = value

    @builtins.property
    def button_triggered(self):
        """Message field 'button_triggered'."""
        return self._button_triggered

    @button_triggered.setter
    def button_triggered(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'button_triggered' field must be of type 'bool'"
        self._button_triggered = value

    @builtins.property
    def validation_performed(self):
        """Message field 'validation_performed'."""
        return self._validation_performed

    @validation_performed.setter
    def validation_performed(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'validation_performed' field must be of type 'bool'"
        self._validation_performed = value

    @builtins.property
    def operation_executed(self):
        """Message field 'operation_executed'."""
        return self._operation_executed

    @operation_executed.setter
    def operation_executed(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'operation_executed' field must be of type 'bool'"
        self._operation_executed = value

    @builtins.property
    def diagnostic_stage(self):
        """Message field 'diagnostic_stage'."""
        return self._diagnostic_stage

    @diagnostic_stage.setter
    def diagnostic_stage(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'diagnostic_stage' field must be of type 'str'"
        self._diagnostic_stage = value

    @builtins.property
    def path_fraction(self):
        """Message field 'path_fraction'."""
        return self._path_fraction

    @path_fraction.setter
    def path_fraction(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'path_fraction' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'path_fraction' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._path_fraction = value

    @builtins.property
    def required_fraction(self):
        """Message field 'required_fraction'."""
        return self._required_fraction

    @required_fraction.setter
    def required_fraction(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'required_fraction' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'required_fraction' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._required_fraction = value

    @builtins.property
    def moveit_error_code(self):
        """Message field 'moveit_error_code'."""
        return self._moveit_error_code

    @moveit_error_code.setter
    def moveit_error_code(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'moveit_error_code' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'moveit_error_code' field must be an integer in [-2147483648, 2147483647]"
        self._moveit_error_code = value

    @builtins.property
    def policy_reason(self):
        """Message field 'policy_reason'."""
        return self._policy_reason

    @policy_reason.setter
    def policy_reason(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'policy_reason' field must be of type 'str'"
        self._policy_reason = value

    @builtins.property
    def failure_reason(self):
        """Message field 'failure_reason'."""
        return self._failure_reason

    @failure_reason.setter
    def failure_reason(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'failure_reason' field must be of type 'str'"
        self._failure_reason = value

    @builtins.property
    def physical_outcome_confirmed(self):
        """Message field 'physical_outcome_confirmed'."""
        return self._physical_outcome_confirmed

    @physical_outcome_confirmed.setter
    def physical_outcome_confirmed(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'physical_outcome_confirmed' field must be of type 'bool'"
        self._physical_outcome_confirmed = value

    @builtins.property
    def final_state_verified(self):
        """Message field 'final_state_verified'."""
        return self._final_state_verified

    @final_state_verified.setter
    def final_state_verified(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'final_state_verified' field must be of type 'bool'"
        self._final_state_verified = value

    @builtins.property
    def transport_succeeded(self):
        """Message field 'transport_succeeded'."""
        return self._transport_succeeded

    @transport_succeeded.setter
    def transport_succeeded(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'transport_succeeded' field must be of type 'bool'"
        self._transport_succeeded = value

    @builtins.property
    def recovery_succeeded(self):
        """Message field 'recovery_succeeded'."""
        return self._recovery_succeeded

    @recovery_succeeded.setter
    def recovery_succeeded(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'recovery_succeeded' field must be of type 'bool'"
        self._recovery_succeeded = value

    @builtins.property
    def grasp_released(self):
        """Message field 'grasp_released'."""
        return self._grasp_released

    @grasp_released.setter
    def grasp_released(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'grasp_released' field must be of type 'bool'"
        self._grasp_released = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import math

# already imported above
# import rosidl_parser.definition


class Metaclass_OperateCabinetControl_Feedback(type):
    """Metaclass of message 'OperateCabinetControl_Feedback'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'WAITING_FOR_SYSTEM': 0,
        'NAVIGATING': 1,
        'DOCKING': 2,
        'MOVING_TO_READY': 3,
        'APPROACHING': 4,
        'GRASPING': 5,
        'MANIPULATING': 6,
        'VERIFYING': 7,
        'RELEASING': 8,
        'RETREATING': 9,
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
                'xczs_inspection_robot_interfaces.action.OperateCabinetControl_Feedback')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__operate_cabinet_control__feedback
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__operate_cabinet_control__feedback
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__operate_cabinet_control__feedback
            cls._TYPE_SUPPORT = module.type_support_msg__action__operate_cabinet_control__feedback
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__operate_cabinet_control__feedback

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'WAITING_FOR_SYSTEM': cls.__constants['WAITING_FOR_SYSTEM'],
            'NAVIGATING': cls.__constants['NAVIGATING'],
            'DOCKING': cls.__constants['DOCKING'],
            'MOVING_TO_READY': cls.__constants['MOVING_TO_READY'],
            'APPROACHING': cls.__constants['APPROACHING'],
            'GRASPING': cls.__constants['GRASPING'],
            'MANIPULATING': cls.__constants['MANIPULATING'],
            'VERIFYING': cls.__constants['VERIFYING'],
            'RELEASING': cls.__constants['RELEASING'],
            'RETREATING': cls.__constants['RETREATING'],
        }

    @property
    def WAITING_FOR_SYSTEM(self):
        """Message constant 'WAITING_FOR_SYSTEM'."""
        return Metaclass_OperateCabinetControl_Feedback.__constants['WAITING_FOR_SYSTEM']

    @property
    def NAVIGATING(self):
        """Message constant 'NAVIGATING'."""
        return Metaclass_OperateCabinetControl_Feedback.__constants['NAVIGATING']

    @property
    def DOCKING(self):
        """Message constant 'DOCKING'."""
        return Metaclass_OperateCabinetControl_Feedback.__constants['DOCKING']

    @property
    def MOVING_TO_READY(self):
        """Message constant 'MOVING_TO_READY'."""
        return Metaclass_OperateCabinetControl_Feedback.__constants['MOVING_TO_READY']

    @property
    def APPROACHING(self):
        """Message constant 'APPROACHING'."""
        return Metaclass_OperateCabinetControl_Feedback.__constants['APPROACHING']

    @property
    def GRASPING(self):
        """Message constant 'GRASPING'."""
        return Metaclass_OperateCabinetControl_Feedback.__constants['GRASPING']

    @property
    def MANIPULATING(self):
        """Message constant 'MANIPULATING'."""
        return Metaclass_OperateCabinetControl_Feedback.__constants['MANIPULATING']

    @property
    def VERIFYING(self):
        """Message constant 'VERIFYING'."""
        return Metaclass_OperateCabinetControl_Feedback.__constants['VERIFYING']

    @property
    def RELEASING(self):
        """Message constant 'RELEASING'."""
        return Metaclass_OperateCabinetControl_Feedback.__constants['RELEASING']

    @property
    def RETREATING(self):
        """Message constant 'RETREATING'."""
        return Metaclass_OperateCabinetControl_Feedback.__constants['RETREATING']


class OperateCabinetControl_Feedback(metaclass=Metaclass_OperateCabinetControl_Feedback):
    """
    Message class 'OperateCabinetControl_Feedback'.

    Constants:
      WAITING_FOR_SYSTEM
      NAVIGATING
      DOCKING
      MOVING_TO_READY
      APPROACHING
      GRASPING
      MANIPULATING
      VERIFYING
      RELEASING
      RETREATING
    """

    __slots__ = [
        '_phase',
        '_progress',
        '_current_position',
        '_target_position',
        '_current_state',
        '_message',
    ]

    _fields_and_field_types = {
        'phase': 'uint8',
        'progress': 'float',
        'current_position': 'double',
        'target_position': 'double',
        'current_state': 'string',
        'message': 'string',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.phase = kwargs.get('phase', int())
        self.progress = kwargs.get('progress', float())
        self.current_position = kwargs.get('current_position', float())
        self.target_position = kwargs.get('target_position', float())
        self.current_state = kwargs.get('current_state', str())
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
        if self.phase != other.phase:
            return False
        if self.progress != other.progress:
            return False
        if self.current_position != other.current_position:
            return False
        if self.target_position != other.target_position:
            return False
        if self.current_state != other.current_state:
            return False
        if self.message != other.message:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def phase(self):
        """Message field 'phase'."""
        return self._phase

    @phase.setter
    def phase(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'phase' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'phase' field must be an unsigned integer in [0, 255]"
        self._phase = value

    @builtins.property
    def progress(self):
        """Message field 'progress'."""
        return self._progress

    @progress.setter
    def progress(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'progress' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'progress' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._progress = value

    @builtins.property
    def current_position(self):
        """Message field 'current_position'."""
        return self._current_position

    @current_position.setter
    def current_position(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'current_position' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'current_position' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._current_position = value

    @builtins.property
    def target_position(self):
        """Message field 'target_position'."""
        return self._target_position

    @target_position.setter
    def target_position(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'target_position' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'target_position' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._target_position = value

    @builtins.property
    def current_state(self):
        """Message field 'current_state'."""
        return self._current_state

    @current_state.setter
    def current_state(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'current_state' field must be of type 'str'"
        self._current_state = value

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


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_OperateCabinetControl_SendGoal_Request(type):
    """Metaclass of message 'OperateCabinetControl_SendGoal_Request'."""

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
                'xczs_inspection_robot_interfaces.action.OperateCabinetControl_SendGoal_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__operate_cabinet_control__send_goal__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__operate_cabinet_control__send_goal__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__operate_cabinet_control__send_goal__request
            cls._TYPE_SUPPORT = module.type_support_msg__action__operate_cabinet_control__send_goal__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__operate_cabinet_control__send_goal__request

            from unique_identifier_msgs.msg import UUID
            if UUID.__class__._TYPE_SUPPORT is None:
                UUID.__class__.__import_type_support__()

            from xczs_inspection_robot_interfaces.action import OperateCabinetControl
            if OperateCabinetControl.Goal.__class__._TYPE_SUPPORT is None:
                OperateCabinetControl.Goal.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class OperateCabinetControl_SendGoal_Request(metaclass=Metaclass_OperateCabinetControl_SendGoal_Request):
    """Message class 'OperateCabinetControl_SendGoal_Request'."""

    __slots__ = [
        '_goal_id',
        '_goal',
    ]

    _fields_and_field_types = {
        'goal_id': 'unique_identifier_msgs/UUID',
        'goal': 'xczs_inspection_robot_interfaces/OperateCabinetControl_Goal',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['unique_identifier_msgs', 'msg'], 'UUID'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['xczs_inspection_robot_interfaces', 'action'], 'OperateCabinetControl_Goal'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from unique_identifier_msgs.msg import UUID
        self.goal_id = kwargs.get('goal_id', UUID())
        from xczs_inspection_robot_interfaces.action._operate_cabinet_control import OperateCabinetControl_Goal
        self.goal = kwargs.get('goal', OperateCabinetControl_Goal())

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
        if self.goal_id != other.goal_id:
            return False
        if self.goal != other.goal:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def goal_id(self):
        """Message field 'goal_id'."""
        return self._goal_id

    @goal_id.setter
    def goal_id(self, value):
        if __debug__:
            from unique_identifier_msgs.msg import UUID
            assert \
                isinstance(value, UUID), \
                "The 'goal_id' field must be a sub message of type 'UUID'"
        self._goal_id = value

    @builtins.property
    def goal(self):
        """Message field 'goal'."""
        return self._goal

    @goal.setter
    def goal(self, value):
        if __debug__:
            from xczs_inspection_robot_interfaces.action._operate_cabinet_control import OperateCabinetControl_Goal
            assert \
                isinstance(value, OperateCabinetControl_Goal), \
                "The 'goal' field must be a sub message of type 'OperateCabinetControl_Goal'"
        self._goal = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_OperateCabinetControl_SendGoal_Response(type):
    """Metaclass of message 'OperateCabinetControl_SendGoal_Response'."""

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
                'xczs_inspection_robot_interfaces.action.OperateCabinetControl_SendGoal_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__operate_cabinet_control__send_goal__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__operate_cabinet_control__send_goal__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__operate_cabinet_control__send_goal__response
            cls._TYPE_SUPPORT = module.type_support_msg__action__operate_cabinet_control__send_goal__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__operate_cabinet_control__send_goal__response

            from builtin_interfaces.msg import Time
            if Time.__class__._TYPE_SUPPORT is None:
                Time.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class OperateCabinetControl_SendGoal_Response(metaclass=Metaclass_OperateCabinetControl_SendGoal_Response):
    """Message class 'OperateCabinetControl_SendGoal_Response'."""

    __slots__ = [
        '_accepted',
        '_stamp',
    ]

    _fields_and_field_types = {
        'accepted': 'boolean',
        'stamp': 'builtin_interfaces/Time',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['builtin_interfaces', 'msg'], 'Time'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.accepted = kwargs.get('accepted', bool())
        from builtin_interfaces.msg import Time
        self.stamp = kwargs.get('stamp', Time())

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
        if self.stamp != other.stamp:
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
    def stamp(self):
        """Message field 'stamp'."""
        return self._stamp

    @stamp.setter
    def stamp(self, value):
        if __debug__:
            from builtin_interfaces.msg import Time
            assert \
                isinstance(value, Time), \
                "The 'stamp' field must be a sub message of type 'Time'"
        self._stamp = value


class Metaclass_OperateCabinetControl_SendGoal(type):
    """Metaclass of service 'OperateCabinetControl_SendGoal'."""

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
                'xczs_inspection_robot_interfaces.action.OperateCabinetControl_SendGoal')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__action__operate_cabinet_control__send_goal

            from xczs_inspection_robot_interfaces.action import _operate_cabinet_control
            if _operate_cabinet_control.Metaclass_OperateCabinetControl_SendGoal_Request._TYPE_SUPPORT is None:
                _operate_cabinet_control.Metaclass_OperateCabinetControl_SendGoal_Request.__import_type_support__()
            if _operate_cabinet_control.Metaclass_OperateCabinetControl_SendGoal_Response._TYPE_SUPPORT is None:
                _operate_cabinet_control.Metaclass_OperateCabinetControl_SendGoal_Response.__import_type_support__()


class OperateCabinetControl_SendGoal(metaclass=Metaclass_OperateCabinetControl_SendGoal):
    from xczs_inspection_robot_interfaces.action._operate_cabinet_control import OperateCabinetControl_SendGoal_Request as Request
    from xczs_inspection_robot_interfaces.action._operate_cabinet_control import OperateCabinetControl_SendGoal_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_OperateCabinetControl_GetResult_Request(type):
    """Metaclass of message 'OperateCabinetControl_GetResult_Request'."""

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
                'xczs_inspection_robot_interfaces.action.OperateCabinetControl_GetResult_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__operate_cabinet_control__get_result__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__operate_cabinet_control__get_result__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__operate_cabinet_control__get_result__request
            cls._TYPE_SUPPORT = module.type_support_msg__action__operate_cabinet_control__get_result__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__operate_cabinet_control__get_result__request

            from unique_identifier_msgs.msg import UUID
            if UUID.__class__._TYPE_SUPPORT is None:
                UUID.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class OperateCabinetControl_GetResult_Request(metaclass=Metaclass_OperateCabinetControl_GetResult_Request):
    """Message class 'OperateCabinetControl_GetResult_Request'."""

    __slots__ = [
        '_goal_id',
    ]

    _fields_and_field_types = {
        'goal_id': 'unique_identifier_msgs/UUID',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['unique_identifier_msgs', 'msg'], 'UUID'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from unique_identifier_msgs.msg import UUID
        self.goal_id = kwargs.get('goal_id', UUID())

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
        if self.goal_id != other.goal_id:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def goal_id(self):
        """Message field 'goal_id'."""
        return self._goal_id

    @goal_id.setter
    def goal_id(self, value):
        if __debug__:
            from unique_identifier_msgs.msg import UUID
            assert \
                isinstance(value, UUID), \
                "The 'goal_id' field must be a sub message of type 'UUID'"
        self._goal_id = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_OperateCabinetControl_GetResult_Response(type):
    """Metaclass of message 'OperateCabinetControl_GetResult_Response'."""

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
                'xczs_inspection_robot_interfaces.action.OperateCabinetControl_GetResult_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__operate_cabinet_control__get_result__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__operate_cabinet_control__get_result__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__operate_cabinet_control__get_result__response
            cls._TYPE_SUPPORT = module.type_support_msg__action__operate_cabinet_control__get_result__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__operate_cabinet_control__get_result__response

            from xczs_inspection_robot_interfaces.action import OperateCabinetControl
            if OperateCabinetControl.Result.__class__._TYPE_SUPPORT is None:
                OperateCabinetControl.Result.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class OperateCabinetControl_GetResult_Response(metaclass=Metaclass_OperateCabinetControl_GetResult_Response):
    """Message class 'OperateCabinetControl_GetResult_Response'."""

    __slots__ = [
        '_status',
        '_result',
    ]

    _fields_and_field_types = {
        'status': 'int8',
        'result': 'xczs_inspection_robot_interfaces/OperateCabinetControl_Result',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('int8'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['xczs_inspection_robot_interfaces', 'action'], 'OperateCabinetControl_Result'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.status = kwargs.get('status', int())
        from xczs_inspection_robot_interfaces.action._operate_cabinet_control import OperateCabinetControl_Result
        self.result = kwargs.get('result', OperateCabinetControl_Result())

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
        if self.status != other.status:
            return False
        if self.result != other.result:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def status(self):
        """Message field 'status'."""
        return self._status

    @status.setter
    def status(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'status' field must be of type 'int'"
            assert value >= -128 and value < 128, \
                "The 'status' field must be an integer in [-128, 127]"
        self._status = value

    @builtins.property
    def result(self):
        """Message field 'result'."""
        return self._result

    @result.setter
    def result(self, value):
        if __debug__:
            from xczs_inspection_robot_interfaces.action._operate_cabinet_control import OperateCabinetControl_Result
            assert \
                isinstance(value, OperateCabinetControl_Result), \
                "The 'result' field must be a sub message of type 'OperateCabinetControl_Result'"
        self._result = value


class Metaclass_OperateCabinetControl_GetResult(type):
    """Metaclass of service 'OperateCabinetControl_GetResult'."""

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
                'xczs_inspection_robot_interfaces.action.OperateCabinetControl_GetResult')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__action__operate_cabinet_control__get_result

            from xczs_inspection_robot_interfaces.action import _operate_cabinet_control
            if _operate_cabinet_control.Metaclass_OperateCabinetControl_GetResult_Request._TYPE_SUPPORT is None:
                _operate_cabinet_control.Metaclass_OperateCabinetControl_GetResult_Request.__import_type_support__()
            if _operate_cabinet_control.Metaclass_OperateCabinetControl_GetResult_Response._TYPE_SUPPORT is None:
                _operate_cabinet_control.Metaclass_OperateCabinetControl_GetResult_Response.__import_type_support__()


class OperateCabinetControl_GetResult(metaclass=Metaclass_OperateCabinetControl_GetResult):
    from xczs_inspection_robot_interfaces.action._operate_cabinet_control import OperateCabinetControl_GetResult_Request as Request
    from xczs_inspection_robot_interfaces.action._operate_cabinet_control import OperateCabinetControl_GetResult_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_OperateCabinetControl_FeedbackMessage(type):
    """Metaclass of message 'OperateCabinetControl_FeedbackMessage'."""

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
                'xczs_inspection_robot_interfaces.action.OperateCabinetControl_FeedbackMessage')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__operate_cabinet_control__feedback_message
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__operate_cabinet_control__feedback_message
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__operate_cabinet_control__feedback_message
            cls._TYPE_SUPPORT = module.type_support_msg__action__operate_cabinet_control__feedback_message
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__operate_cabinet_control__feedback_message

            from unique_identifier_msgs.msg import UUID
            if UUID.__class__._TYPE_SUPPORT is None:
                UUID.__class__.__import_type_support__()

            from xczs_inspection_robot_interfaces.action import OperateCabinetControl
            if OperateCabinetControl.Feedback.__class__._TYPE_SUPPORT is None:
                OperateCabinetControl.Feedback.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class OperateCabinetControl_FeedbackMessage(metaclass=Metaclass_OperateCabinetControl_FeedbackMessage):
    """Message class 'OperateCabinetControl_FeedbackMessage'."""

    __slots__ = [
        '_goal_id',
        '_feedback',
    ]

    _fields_and_field_types = {
        'goal_id': 'unique_identifier_msgs/UUID',
        'feedback': 'xczs_inspection_robot_interfaces/OperateCabinetControl_Feedback',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['unique_identifier_msgs', 'msg'], 'UUID'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['xczs_inspection_robot_interfaces', 'action'], 'OperateCabinetControl_Feedback'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from unique_identifier_msgs.msg import UUID
        self.goal_id = kwargs.get('goal_id', UUID())
        from xczs_inspection_robot_interfaces.action._operate_cabinet_control import OperateCabinetControl_Feedback
        self.feedback = kwargs.get('feedback', OperateCabinetControl_Feedback())

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
        if self.goal_id != other.goal_id:
            return False
        if self.feedback != other.feedback:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def goal_id(self):
        """Message field 'goal_id'."""
        return self._goal_id

    @goal_id.setter
    def goal_id(self, value):
        if __debug__:
            from unique_identifier_msgs.msg import UUID
            assert \
                isinstance(value, UUID), \
                "The 'goal_id' field must be a sub message of type 'UUID'"
        self._goal_id = value

    @builtins.property
    def feedback(self):
        """Message field 'feedback'."""
        return self._feedback

    @feedback.setter
    def feedback(self, value):
        if __debug__:
            from xczs_inspection_robot_interfaces.action._operate_cabinet_control import OperateCabinetControl_Feedback
            assert \
                isinstance(value, OperateCabinetControl_Feedback), \
                "The 'feedback' field must be a sub message of type 'OperateCabinetControl_Feedback'"
        self._feedback = value


class Metaclass_OperateCabinetControl(type):
    """Metaclass of action 'OperateCabinetControl'."""

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
                'xczs_inspection_robot_interfaces.action.OperateCabinetControl')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_action__action__operate_cabinet_control

            from action_msgs.msg import _goal_status_array
            if _goal_status_array.Metaclass_GoalStatusArray._TYPE_SUPPORT is None:
                _goal_status_array.Metaclass_GoalStatusArray.__import_type_support__()
            from action_msgs.srv import _cancel_goal
            if _cancel_goal.Metaclass_CancelGoal._TYPE_SUPPORT is None:
                _cancel_goal.Metaclass_CancelGoal.__import_type_support__()

            from xczs_inspection_robot_interfaces.action import _operate_cabinet_control
            if _operate_cabinet_control.Metaclass_OperateCabinetControl_SendGoal._TYPE_SUPPORT is None:
                _operate_cabinet_control.Metaclass_OperateCabinetControl_SendGoal.__import_type_support__()
            if _operate_cabinet_control.Metaclass_OperateCabinetControl_GetResult._TYPE_SUPPORT is None:
                _operate_cabinet_control.Metaclass_OperateCabinetControl_GetResult.__import_type_support__()
            if _operate_cabinet_control.Metaclass_OperateCabinetControl_FeedbackMessage._TYPE_SUPPORT is None:
                _operate_cabinet_control.Metaclass_OperateCabinetControl_FeedbackMessage.__import_type_support__()


class OperateCabinetControl(metaclass=Metaclass_OperateCabinetControl):

    # The goal message defined in the action definition.
    from xczs_inspection_robot_interfaces.action._operate_cabinet_control import OperateCabinetControl_Goal as Goal
    # The result message defined in the action definition.
    from xczs_inspection_robot_interfaces.action._operate_cabinet_control import OperateCabinetControl_Result as Result
    # The feedback message defined in the action definition.
    from xczs_inspection_robot_interfaces.action._operate_cabinet_control import OperateCabinetControl_Feedback as Feedback

    class Impl:

        # The send_goal service using a wrapped version of the goal message as a request.
        from xczs_inspection_robot_interfaces.action._operate_cabinet_control import OperateCabinetControl_SendGoal as SendGoalService
        # The get_result service using a wrapped version of the result message as a response.
        from xczs_inspection_robot_interfaces.action._operate_cabinet_control import OperateCabinetControl_GetResult as GetResultService
        # The feedback message with generic fields which wraps the feedback message.
        from xczs_inspection_robot_interfaces.action._operate_cabinet_control import OperateCabinetControl_FeedbackMessage as FeedbackMessage

        # The generic service to cancel a goal.
        from action_msgs.srv._cancel_goal import CancelGoal as CancelGoalService
        # The generic message for get the status of a goal.
        from action_msgs.msg._goal_status_array import GoalStatusArray as GoalStatusMessage

    def __init__(self):
        raise NotImplementedError('Action classes can not be instantiated')

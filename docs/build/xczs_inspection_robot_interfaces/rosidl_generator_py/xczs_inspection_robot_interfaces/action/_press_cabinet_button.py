# generated from rosidl_generator_py/resource/_idl.py.em
# with input from xczs_inspection_robot_interfaces:action/PressCabinetButton.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_PressCabinetButton_Goal(type):
    """Metaclass of message 'PressCabinetButton_Goal'."""

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
                'xczs_inspection_robot_interfaces.action.PressCabinetButton_Goal')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__press_cabinet_button__goal
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__press_cabinet_button__goal
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__press_cabinet_button__goal
            cls._TYPE_SUPPORT = module.type_support_msg__action__press_cabinet_button__goal
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__press_cabinet_button__goal

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class PressCabinetButton_Goal(metaclass=Metaclass_PressCabinetButton_Goal):
    """Message class 'PressCabinetButton_Goal'."""

    __slots__ = [
        '_button_id',
        '_navigate_to_staging_pose',
    ]

    _fields_and_field_types = {
        'button_id': 'string',
        'navigate_to_staging_pose': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.button_id = kwargs.get('button_id', str())
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
        if self.button_id != other.button_id:
            return False
        if self.navigate_to_staging_pose != other.navigate_to_staging_pose:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def button_id(self):
        """Message field 'button_id'."""
        return self._button_id

    @button_id.setter
    def button_id(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'button_id' field must be of type 'str'"
        self._button_id = value

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

import math  # noqa: E402, I100

# already imported above
# import rosidl_parser.definition


class Metaclass_PressCabinetButton_Result(type):
    """Metaclass of message 'PressCabinetButton_Result'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'SUCCESS': 0,
        'INVALID_BUTTON': 1,
        'NOT_READY': 2,
        'NAVIGATION_FAILED': 3,
        'PLANNING_FAILED': 4,
        'EXECUTION_FAILED': 5,
        'PRESS_NOT_DETECTED': 6,
        'RELEASE_NOT_DETECTED': 7,
        'CANCELED': 8,
        'INTERNAL_ERROR': 9,
        'RESOURCE_BUSY': 10,
        'LEASE_LOST': 11,
        'TOOLSET_MISMATCH': 12,
        'ADAPTER_NOT_VALIDATED': 13,
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
                'xczs_inspection_robot_interfaces.action.PressCabinetButton_Result')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__press_cabinet_button__result
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__press_cabinet_button__result
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__press_cabinet_button__result
            cls._TYPE_SUPPORT = module.type_support_msg__action__press_cabinet_button__result
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__press_cabinet_button__result

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'SUCCESS': cls.__constants['SUCCESS'],
            'INVALID_BUTTON': cls.__constants['INVALID_BUTTON'],
            'NOT_READY': cls.__constants['NOT_READY'],
            'NAVIGATION_FAILED': cls.__constants['NAVIGATION_FAILED'],
            'PLANNING_FAILED': cls.__constants['PLANNING_FAILED'],
            'EXECUTION_FAILED': cls.__constants['EXECUTION_FAILED'],
            'PRESS_NOT_DETECTED': cls.__constants['PRESS_NOT_DETECTED'],
            'RELEASE_NOT_DETECTED': cls.__constants['RELEASE_NOT_DETECTED'],
            'CANCELED': cls.__constants['CANCELED'],
            'INTERNAL_ERROR': cls.__constants['INTERNAL_ERROR'],
            'RESOURCE_BUSY': cls.__constants['RESOURCE_BUSY'],
            'LEASE_LOST': cls.__constants['LEASE_LOST'],
            'TOOLSET_MISMATCH': cls.__constants['TOOLSET_MISMATCH'],
            'ADAPTER_NOT_VALIDATED': cls.__constants['ADAPTER_NOT_VALIDATED'],
        }

    @property
    def SUCCESS(self):
        """Message constant 'SUCCESS'."""
        return Metaclass_PressCabinetButton_Result.__constants['SUCCESS']

    @property
    def INVALID_BUTTON(self):
        """Message constant 'INVALID_BUTTON'."""
        return Metaclass_PressCabinetButton_Result.__constants['INVALID_BUTTON']

    @property
    def NOT_READY(self):
        """Message constant 'NOT_READY'."""
        return Metaclass_PressCabinetButton_Result.__constants['NOT_READY']

    @property
    def NAVIGATION_FAILED(self):
        """Message constant 'NAVIGATION_FAILED'."""
        return Metaclass_PressCabinetButton_Result.__constants['NAVIGATION_FAILED']

    @property
    def PLANNING_FAILED(self):
        """Message constant 'PLANNING_FAILED'."""
        return Metaclass_PressCabinetButton_Result.__constants['PLANNING_FAILED']

    @property
    def EXECUTION_FAILED(self):
        """Message constant 'EXECUTION_FAILED'."""
        return Metaclass_PressCabinetButton_Result.__constants['EXECUTION_FAILED']

    @property
    def PRESS_NOT_DETECTED(self):
        """Message constant 'PRESS_NOT_DETECTED'."""
        return Metaclass_PressCabinetButton_Result.__constants['PRESS_NOT_DETECTED']

    @property
    def RELEASE_NOT_DETECTED(self):
        """Message constant 'RELEASE_NOT_DETECTED'."""
        return Metaclass_PressCabinetButton_Result.__constants['RELEASE_NOT_DETECTED']

    @property
    def CANCELED(self):
        """Message constant 'CANCELED'."""
        return Metaclass_PressCabinetButton_Result.__constants['CANCELED']

    @property
    def INTERNAL_ERROR(self):
        """Message constant 'INTERNAL_ERROR'."""
        return Metaclass_PressCabinetButton_Result.__constants['INTERNAL_ERROR']

    @property
    def RESOURCE_BUSY(self):
        """Message constant 'RESOURCE_BUSY'."""
        return Metaclass_PressCabinetButton_Result.__constants['RESOURCE_BUSY']

    @property
    def LEASE_LOST(self):
        """Message constant 'LEASE_LOST'."""
        return Metaclass_PressCabinetButton_Result.__constants['LEASE_LOST']

    @property
    def TOOLSET_MISMATCH(self):
        """Message constant 'TOOLSET_MISMATCH'."""
        return Metaclass_PressCabinetButton_Result.__constants['TOOLSET_MISMATCH']

    @property
    def ADAPTER_NOT_VALIDATED(self):
        """Message constant 'ADAPTER_NOT_VALIDATED'."""
        return Metaclass_PressCabinetButton_Result.__constants['ADAPTER_NOT_VALIDATED']


class PressCabinetButton_Result(metaclass=Metaclass_PressCabinetButton_Result):
    """
    Message class 'PressCabinetButton_Result'.

    Constants:
      SUCCESS
      INVALID_BUTTON
      NOT_READY
      NAVIGATION_FAILED
      PLANNING_FAILED
      EXECUTION_FAILED
      PRESS_NOT_DETECTED
      RELEASE_NOT_DETECTED
      CANCELED
      INTERNAL_ERROR
      RESOURCE_BUSY
      LEASE_LOST
      TOOLSET_MISMATCH
      ADAPTER_NOT_VALIDATED
    """

    __slots__ = [
        '_success',
        '_error_code',
        '_message',
        '_max_travel',
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
        'max_travel': 'double',
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
        self.max_travel = kwargs.get('max_travel', float())
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
        if self.max_travel != other.max_travel:
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
    def max_travel(self):
        """Message field 'max_travel'."""
        return self._max_travel

    @max_travel.setter
    def max_travel(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'max_travel' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'max_travel' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._max_travel = value

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


class Metaclass_PressCabinetButton_Feedback(type):
    """Metaclass of message 'PressCabinetButton_Feedback'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'WAITING_FOR_SYSTEM': 0,
        'NAVIGATING': 1,
        'MOVING_TO_PREPRESS': 2,
        'APPROACHING': 3,
        'PRESSING': 4,
        'VERIFYING_PRESS': 5,
        'RETRACTING': 6,
        'VERIFYING_RELEASE': 7,
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
                'xczs_inspection_robot_interfaces.action.PressCabinetButton_Feedback')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__press_cabinet_button__feedback
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__press_cabinet_button__feedback
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__press_cabinet_button__feedback
            cls._TYPE_SUPPORT = module.type_support_msg__action__press_cabinet_button__feedback
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__press_cabinet_button__feedback

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'WAITING_FOR_SYSTEM': cls.__constants['WAITING_FOR_SYSTEM'],
            'NAVIGATING': cls.__constants['NAVIGATING'],
            'MOVING_TO_PREPRESS': cls.__constants['MOVING_TO_PREPRESS'],
            'APPROACHING': cls.__constants['APPROACHING'],
            'PRESSING': cls.__constants['PRESSING'],
            'VERIFYING_PRESS': cls.__constants['VERIFYING_PRESS'],
            'RETRACTING': cls.__constants['RETRACTING'],
            'VERIFYING_RELEASE': cls.__constants['VERIFYING_RELEASE'],
        }

    @property
    def WAITING_FOR_SYSTEM(self):
        """Message constant 'WAITING_FOR_SYSTEM'."""
        return Metaclass_PressCabinetButton_Feedback.__constants['WAITING_FOR_SYSTEM']

    @property
    def NAVIGATING(self):
        """Message constant 'NAVIGATING'."""
        return Metaclass_PressCabinetButton_Feedback.__constants['NAVIGATING']

    @property
    def MOVING_TO_PREPRESS(self):
        """Message constant 'MOVING_TO_PREPRESS'."""
        return Metaclass_PressCabinetButton_Feedback.__constants['MOVING_TO_PREPRESS']

    @property
    def APPROACHING(self):
        """Message constant 'APPROACHING'."""
        return Metaclass_PressCabinetButton_Feedback.__constants['APPROACHING']

    @property
    def PRESSING(self):
        """Message constant 'PRESSING'."""
        return Metaclass_PressCabinetButton_Feedback.__constants['PRESSING']

    @property
    def VERIFYING_PRESS(self):
        """Message constant 'VERIFYING_PRESS'."""
        return Metaclass_PressCabinetButton_Feedback.__constants['VERIFYING_PRESS']

    @property
    def RETRACTING(self):
        """Message constant 'RETRACTING'."""
        return Metaclass_PressCabinetButton_Feedback.__constants['RETRACTING']

    @property
    def VERIFYING_RELEASE(self):
        """Message constant 'VERIFYING_RELEASE'."""
        return Metaclass_PressCabinetButton_Feedback.__constants['VERIFYING_RELEASE']


class PressCabinetButton_Feedback(metaclass=Metaclass_PressCabinetButton_Feedback):
    """
    Message class 'PressCabinetButton_Feedback'.

    Constants:
      WAITING_FOR_SYSTEM
      NAVIGATING
      MOVING_TO_PREPRESS
      APPROACHING
      PRESSING
      VERIFYING_PRESS
      RETRACTING
      VERIFYING_RELEASE
    """

    __slots__ = [
        '_phase',
        '_progress',
        '_button_travel',
        '_message',
    ]

    _fields_and_field_types = {
        'phase': 'uint8',
        'progress': 'float',
        'button_travel': 'double',
        'message': 'string',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.phase = kwargs.get('phase', int())
        self.progress = kwargs.get('progress', float())
        self.button_travel = kwargs.get('button_travel', float())
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
        if self.button_travel != other.button_travel:
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
    def button_travel(self):
        """Message field 'button_travel'."""
        return self._button_travel

    @button_travel.setter
    def button_travel(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'button_travel' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'button_travel' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._button_travel = value

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


class Metaclass_PressCabinetButton_SendGoal_Request(type):
    """Metaclass of message 'PressCabinetButton_SendGoal_Request'."""

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
                'xczs_inspection_robot_interfaces.action.PressCabinetButton_SendGoal_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__press_cabinet_button__send_goal__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__press_cabinet_button__send_goal__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__press_cabinet_button__send_goal__request
            cls._TYPE_SUPPORT = module.type_support_msg__action__press_cabinet_button__send_goal__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__press_cabinet_button__send_goal__request

            from unique_identifier_msgs.msg import UUID
            if UUID.__class__._TYPE_SUPPORT is None:
                UUID.__class__.__import_type_support__()

            from xczs_inspection_robot_interfaces.action import PressCabinetButton
            if PressCabinetButton.Goal.__class__._TYPE_SUPPORT is None:
                PressCabinetButton.Goal.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class PressCabinetButton_SendGoal_Request(metaclass=Metaclass_PressCabinetButton_SendGoal_Request):
    """Message class 'PressCabinetButton_SendGoal_Request'."""

    __slots__ = [
        '_goal_id',
        '_goal',
    ]

    _fields_and_field_types = {
        'goal_id': 'unique_identifier_msgs/UUID',
        'goal': 'xczs_inspection_robot_interfaces/PressCabinetButton_Goal',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['unique_identifier_msgs', 'msg'], 'UUID'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['xczs_inspection_robot_interfaces', 'action'], 'PressCabinetButton_Goal'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from unique_identifier_msgs.msg import UUID
        self.goal_id = kwargs.get('goal_id', UUID())
        from xczs_inspection_robot_interfaces.action._press_cabinet_button import PressCabinetButton_Goal
        self.goal = kwargs.get('goal', PressCabinetButton_Goal())

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
            from xczs_inspection_robot_interfaces.action._press_cabinet_button import PressCabinetButton_Goal
            assert \
                isinstance(value, PressCabinetButton_Goal), \
                "The 'goal' field must be a sub message of type 'PressCabinetButton_Goal'"
        self._goal = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_PressCabinetButton_SendGoal_Response(type):
    """Metaclass of message 'PressCabinetButton_SendGoal_Response'."""

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
                'xczs_inspection_robot_interfaces.action.PressCabinetButton_SendGoal_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__press_cabinet_button__send_goal__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__press_cabinet_button__send_goal__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__press_cabinet_button__send_goal__response
            cls._TYPE_SUPPORT = module.type_support_msg__action__press_cabinet_button__send_goal__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__press_cabinet_button__send_goal__response

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


class PressCabinetButton_SendGoal_Response(metaclass=Metaclass_PressCabinetButton_SendGoal_Response):
    """Message class 'PressCabinetButton_SendGoal_Response'."""

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


class Metaclass_PressCabinetButton_SendGoal(type):
    """Metaclass of service 'PressCabinetButton_SendGoal'."""

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
                'xczs_inspection_robot_interfaces.action.PressCabinetButton_SendGoal')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__action__press_cabinet_button__send_goal

            from xczs_inspection_robot_interfaces.action import _press_cabinet_button
            if _press_cabinet_button.Metaclass_PressCabinetButton_SendGoal_Request._TYPE_SUPPORT is None:
                _press_cabinet_button.Metaclass_PressCabinetButton_SendGoal_Request.__import_type_support__()
            if _press_cabinet_button.Metaclass_PressCabinetButton_SendGoal_Response._TYPE_SUPPORT is None:
                _press_cabinet_button.Metaclass_PressCabinetButton_SendGoal_Response.__import_type_support__()


class PressCabinetButton_SendGoal(metaclass=Metaclass_PressCabinetButton_SendGoal):
    from xczs_inspection_robot_interfaces.action._press_cabinet_button import PressCabinetButton_SendGoal_Request as Request
    from xczs_inspection_robot_interfaces.action._press_cabinet_button import PressCabinetButton_SendGoal_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_PressCabinetButton_GetResult_Request(type):
    """Metaclass of message 'PressCabinetButton_GetResult_Request'."""

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
                'xczs_inspection_robot_interfaces.action.PressCabinetButton_GetResult_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__press_cabinet_button__get_result__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__press_cabinet_button__get_result__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__press_cabinet_button__get_result__request
            cls._TYPE_SUPPORT = module.type_support_msg__action__press_cabinet_button__get_result__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__press_cabinet_button__get_result__request

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


class PressCabinetButton_GetResult_Request(metaclass=Metaclass_PressCabinetButton_GetResult_Request):
    """Message class 'PressCabinetButton_GetResult_Request'."""

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


class Metaclass_PressCabinetButton_GetResult_Response(type):
    """Metaclass of message 'PressCabinetButton_GetResult_Response'."""

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
                'xczs_inspection_robot_interfaces.action.PressCabinetButton_GetResult_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__press_cabinet_button__get_result__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__press_cabinet_button__get_result__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__press_cabinet_button__get_result__response
            cls._TYPE_SUPPORT = module.type_support_msg__action__press_cabinet_button__get_result__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__press_cabinet_button__get_result__response

            from xczs_inspection_robot_interfaces.action import PressCabinetButton
            if PressCabinetButton.Result.__class__._TYPE_SUPPORT is None:
                PressCabinetButton.Result.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class PressCabinetButton_GetResult_Response(metaclass=Metaclass_PressCabinetButton_GetResult_Response):
    """Message class 'PressCabinetButton_GetResult_Response'."""

    __slots__ = [
        '_status',
        '_result',
    ]

    _fields_and_field_types = {
        'status': 'int8',
        'result': 'xczs_inspection_robot_interfaces/PressCabinetButton_Result',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('int8'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['xczs_inspection_robot_interfaces', 'action'], 'PressCabinetButton_Result'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.status = kwargs.get('status', int())
        from xczs_inspection_robot_interfaces.action._press_cabinet_button import PressCabinetButton_Result
        self.result = kwargs.get('result', PressCabinetButton_Result())

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
            from xczs_inspection_robot_interfaces.action._press_cabinet_button import PressCabinetButton_Result
            assert \
                isinstance(value, PressCabinetButton_Result), \
                "The 'result' field must be a sub message of type 'PressCabinetButton_Result'"
        self._result = value


class Metaclass_PressCabinetButton_GetResult(type):
    """Metaclass of service 'PressCabinetButton_GetResult'."""

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
                'xczs_inspection_robot_interfaces.action.PressCabinetButton_GetResult')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__action__press_cabinet_button__get_result

            from xczs_inspection_robot_interfaces.action import _press_cabinet_button
            if _press_cabinet_button.Metaclass_PressCabinetButton_GetResult_Request._TYPE_SUPPORT is None:
                _press_cabinet_button.Metaclass_PressCabinetButton_GetResult_Request.__import_type_support__()
            if _press_cabinet_button.Metaclass_PressCabinetButton_GetResult_Response._TYPE_SUPPORT is None:
                _press_cabinet_button.Metaclass_PressCabinetButton_GetResult_Response.__import_type_support__()


class PressCabinetButton_GetResult(metaclass=Metaclass_PressCabinetButton_GetResult):
    from xczs_inspection_robot_interfaces.action._press_cabinet_button import PressCabinetButton_GetResult_Request as Request
    from xczs_inspection_robot_interfaces.action._press_cabinet_button import PressCabinetButton_GetResult_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_PressCabinetButton_FeedbackMessage(type):
    """Metaclass of message 'PressCabinetButton_FeedbackMessage'."""

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
                'xczs_inspection_robot_interfaces.action.PressCabinetButton_FeedbackMessage')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__press_cabinet_button__feedback_message
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__press_cabinet_button__feedback_message
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__press_cabinet_button__feedback_message
            cls._TYPE_SUPPORT = module.type_support_msg__action__press_cabinet_button__feedback_message
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__press_cabinet_button__feedback_message

            from unique_identifier_msgs.msg import UUID
            if UUID.__class__._TYPE_SUPPORT is None:
                UUID.__class__.__import_type_support__()

            from xczs_inspection_robot_interfaces.action import PressCabinetButton
            if PressCabinetButton.Feedback.__class__._TYPE_SUPPORT is None:
                PressCabinetButton.Feedback.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class PressCabinetButton_FeedbackMessage(metaclass=Metaclass_PressCabinetButton_FeedbackMessage):
    """Message class 'PressCabinetButton_FeedbackMessage'."""

    __slots__ = [
        '_goal_id',
        '_feedback',
    ]

    _fields_and_field_types = {
        'goal_id': 'unique_identifier_msgs/UUID',
        'feedback': 'xczs_inspection_robot_interfaces/PressCabinetButton_Feedback',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['unique_identifier_msgs', 'msg'], 'UUID'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['xczs_inspection_robot_interfaces', 'action'], 'PressCabinetButton_Feedback'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from unique_identifier_msgs.msg import UUID
        self.goal_id = kwargs.get('goal_id', UUID())
        from xczs_inspection_robot_interfaces.action._press_cabinet_button import PressCabinetButton_Feedback
        self.feedback = kwargs.get('feedback', PressCabinetButton_Feedback())

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
            from xczs_inspection_robot_interfaces.action._press_cabinet_button import PressCabinetButton_Feedback
            assert \
                isinstance(value, PressCabinetButton_Feedback), \
                "The 'feedback' field must be a sub message of type 'PressCabinetButton_Feedback'"
        self._feedback = value


class Metaclass_PressCabinetButton(type):
    """Metaclass of action 'PressCabinetButton'."""

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
                'xczs_inspection_robot_interfaces.action.PressCabinetButton')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_action__action__press_cabinet_button

            from action_msgs.msg import _goal_status_array
            if _goal_status_array.Metaclass_GoalStatusArray._TYPE_SUPPORT is None:
                _goal_status_array.Metaclass_GoalStatusArray.__import_type_support__()
            from action_msgs.srv import _cancel_goal
            if _cancel_goal.Metaclass_CancelGoal._TYPE_SUPPORT is None:
                _cancel_goal.Metaclass_CancelGoal.__import_type_support__()

            from xczs_inspection_robot_interfaces.action import _press_cabinet_button
            if _press_cabinet_button.Metaclass_PressCabinetButton_SendGoal._TYPE_SUPPORT is None:
                _press_cabinet_button.Metaclass_PressCabinetButton_SendGoal.__import_type_support__()
            if _press_cabinet_button.Metaclass_PressCabinetButton_GetResult._TYPE_SUPPORT is None:
                _press_cabinet_button.Metaclass_PressCabinetButton_GetResult.__import_type_support__()
            if _press_cabinet_button.Metaclass_PressCabinetButton_FeedbackMessage._TYPE_SUPPORT is None:
                _press_cabinet_button.Metaclass_PressCabinetButton_FeedbackMessage.__import_type_support__()


class PressCabinetButton(metaclass=Metaclass_PressCabinetButton):

    # The goal message defined in the action definition.
    from xczs_inspection_robot_interfaces.action._press_cabinet_button import PressCabinetButton_Goal as Goal
    # The result message defined in the action definition.
    from xczs_inspection_robot_interfaces.action._press_cabinet_button import PressCabinetButton_Result as Result
    # The feedback message defined in the action definition.
    from xczs_inspection_robot_interfaces.action._press_cabinet_button import PressCabinetButton_Feedback as Feedback

    class Impl:

        # The send_goal service using a wrapped version of the goal message as a request.
        from xczs_inspection_robot_interfaces.action._press_cabinet_button import PressCabinetButton_SendGoal as SendGoalService
        # The get_result service using a wrapped version of the result message as a response.
        from xczs_inspection_robot_interfaces.action._press_cabinet_button import PressCabinetButton_GetResult as GetResultService
        # The feedback message with generic fields which wraps the feedback message.
        from xczs_inspection_robot_interfaces.action._press_cabinet_button import PressCabinetButton_FeedbackMessage as FeedbackMessage

        # The generic service to cancel a goal.
        from action_msgs.srv._cancel_goal import CancelGoal as CancelGoalService
        # The generic message for get the status of a goal.
        from action_msgs.msg._goal_status_array import GoalStatusArray as GoalStatusMessage

    def __init__(self):
        raise NotImplementedError('Action classes can not be instantiated')

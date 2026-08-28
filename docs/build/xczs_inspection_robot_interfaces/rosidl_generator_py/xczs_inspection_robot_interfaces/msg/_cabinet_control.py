# generated from rosidl_generator_py/resource/_idl.py.em
# with input from xczs_inspection_robot_interfaces:msg/CabinetControl.idl
# generated code does not contain a copyright notice


# Import statements for member types

# Member 'state_positions'
import array  # noqa: E402, I100

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_CabinetControl(type):
    """Metaclass of message 'CabinetControl'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'TYPE_BUTTON': 0,
        'TYPE_KNOB': 1,
        'TYPE_SWITCH': 2,
        'TYPE_DOOR': 3,
        'SUPPORT_PRESS': 1,
        'SUPPORT_SET_STATE': 2,
        'SUPPORT_SET_POSITION': 4,
        'SUPPORT_TOGGLE': 8,
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
                'xczs_inspection_robot_interfaces.msg.CabinetControl')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__cabinet_control
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__cabinet_control
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__cabinet_control
            cls._TYPE_SUPPORT = module.type_support_msg__msg__cabinet_control
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__cabinet_control

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'TYPE_BUTTON': cls.__constants['TYPE_BUTTON'],
            'TYPE_KNOB': cls.__constants['TYPE_KNOB'],
            'TYPE_SWITCH': cls.__constants['TYPE_SWITCH'],
            'TYPE_DOOR': cls.__constants['TYPE_DOOR'],
            'SUPPORT_PRESS': cls.__constants['SUPPORT_PRESS'],
            'SUPPORT_SET_STATE': cls.__constants['SUPPORT_SET_STATE'],
            'SUPPORT_SET_POSITION': cls.__constants['SUPPORT_SET_POSITION'],
            'SUPPORT_TOGGLE': cls.__constants['SUPPORT_TOGGLE'],
        }

    @property
    def TYPE_BUTTON(self):
        """Message constant 'TYPE_BUTTON'."""
        return Metaclass_CabinetControl.__constants['TYPE_BUTTON']

    @property
    def TYPE_KNOB(self):
        """Message constant 'TYPE_KNOB'."""
        return Metaclass_CabinetControl.__constants['TYPE_KNOB']

    @property
    def TYPE_SWITCH(self):
        """Message constant 'TYPE_SWITCH'."""
        return Metaclass_CabinetControl.__constants['TYPE_SWITCH']

    @property
    def TYPE_DOOR(self):
        """Message constant 'TYPE_DOOR'."""
        return Metaclass_CabinetControl.__constants['TYPE_DOOR']

    @property
    def SUPPORT_PRESS(self):
        """Message constant 'SUPPORT_PRESS'."""
        return Metaclass_CabinetControl.__constants['SUPPORT_PRESS']

    @property
    def SUPPORT_SET_STATE(self):
        """Message constant 'SUPPORT_SET_STATE'."""
        return Metaclass_CabinetControl.__constants['SUPPORT_SET_STATE']

    @property
    def SUPPORT_SET_POSITION(self):
        """Message constant 'SUPPORT_SET_POSITION'."""
        return Metaclass_CabinetControl.__constants['SUPPORT_SET_POSITION']

    @property
    def SUPPORT_TOGGLE(self):
        """Message constant 'SUPPORT_TOGGLE'."""
        return Metaclass_CabinetControl.__constants['SUPPORT_TOGGLE']


class CabinetControl(metaclass=Metaclass_CabinetControl):
    """
    Message class 'CabinetControl'.

    Constants:
      TYPE_BUTTON
      TYPE_KNOB
      TYPE_SWITCH
      TYPE_DOOR
      SUPPORT_PRESS
      SUPPORT_SET_STATE
      SUPPORT_SET_POSITION
      SUPPORT_TOGGLE
    """

    __slots__ = [
        '_control_id',
        '_display_name',
        '_control_type',
        '_joint_name',
        '_joint_state_topic',
        '_pressed_topic',
        '_state_topic',
        '_supported_commands',
        '_unit',
        '_min_position',
        '_max_position',
        '_state_ids',
        '_state_labels',
        '_state_positions',
        '_requires_grasp',
        '_operable',
        '_unavailable_reason',
        '_required_toolset',
        '_toolset_compatible',
        '_adapter_validated',
        '_default_force',
        '_min_trigger_force',
        '_max_force',
    ]

    _fields_and_field_types = {
        'control_id': 'string',
        'display_name': 'string',
        'control_type': 'uint8',
        'joint_name': 'string',
        'joint_state_topic': 'string',
        'pressed_topic': 'string',
        'state_topic': 'string',
        'supported_commands': 'uint8',
        'unit': 'string',
        'min_position': 'double',
        'max_position': 'double',
        'state_ids': 'sequence<string>',
        'state_labels': 'sequence<string>',
        'state_positions': 'sequence<double>',
        'requires_grasp': 'boolean',
        'operable': 'boolean',
        'unavailable_reason': 'string',
        'required_toolset': 'string',
        'toolset_compatible': 'boolean',
        'adapter_validated': 'boolean',
        'default_force': 'double',
        'min_trigger_force': 'double',
        'max_force': 'double',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.UnboundedString()),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.UnboundedString()),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('double')),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.control_id = kwargs.get('control_id', str())
        self.display_name = kwargs.get('display_name', str())
        self.control_type = kwargs.get('control_type', int())
        self.joint_name = kwargs.get('joint_name', str())
        self.joint_state_topic = kwargs.get('joint_state_topic', str())
        self.pressed_topic = kwargs.get('pressed_topic', str())
        self.state_topic = kwargs.get('state_topic', str())
        self.supported_commands = kwargs.get('supported_commands', int())
        self.unit = kwargs.get('unit', str())
        self.min_position = kwargs.get('min_position', float())
        self.max_position = kwargs.get('max_position', float())
        self.state_ids = kwargs.get('state_ids', [])
        self.state_labels = kwargs.get('state_labels', [])
        self.state_positions = array.array('d', kwargs.get('state_positions', []))
        self.requires_grasp = kwargs.get('requires_grasp', bool())
        self.operable = kwargs.get('operable', bool())
        self.unavailable_reason = kwargs.get('unavailable_reason', str())
        self.required_toolset = kwargs.get('required_toolset', str())
        self.toolset_compatible = kwargs.get('toolset_compatible', bool())
        self.adapter_validated = kwargs.get('adapter_validated', bool())
        self.default_force = kwargs.get('default_force', float())
        self.min_trigger_force = kwargs.get('min_trigger_force', float())
        self.max_force = kwargs.get('max_force', float())

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
        if self.display_name != other.display_name:
            return False
        if self.control_type != other.control_type:
            return False
        if self.joint_name != other.joint_name:
            return False
        if self.joint_state_topic != other.joint_state_topic:
            return False
        if self.pressed_topic != other.pressed_topic:
            return False
        if self.state_topic != other.state_topic:
            return False
        if self.supported_commands != other.supported_commands:
            return False
        if self.unit != other.unit:
            return False
        if self.min_position != other.min_position:
            return False
        if self.max_position != other.max_position:
            return False
        if self.state_ids != other.state_ids:
            return False
        if self.state_labels != other.state_labels:
            return False
        if self.state_positions != other.state_positions:
            return False
        if self.requires_grasp != other.requires_grasp:
            return False
        if self.operable != other.operable:
            return False
        if self.unavailable_reason != other.unavailable_reason:
            return False
        if self.required_toolset != other.required_toolset:
            return False
        if self.toolset_compatible != other.toolset_compatible:
            return False
        if self.adapter_validated != other.adapter_validated:
            return False
        if self.default_force != other.default_force:
            return False
        if self.min_trigger_force != other.min_trigger_force:
            return False
        if self.max_force != other.max_force:
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
    def display_name(self):
        """Message field 'display_name'."""
        return self._display_name

    @display_name.setter
    def display_name(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'display_name' field must be of type 'str'"
        self._display_name = value

    @builtins.property
    def control_type(self):
        """Message field 'control_type'."""
        return self._control_type

    @control_type.setter
    def control_type(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'control_type' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'control_type' field must be an unsigned integer in [0, 255]"
        self._control_type = value

    @builtins.property
    def joint_name(self):
        """Message field 'joint_name'."""
        return self._joint_name

    @joint_name.setter
    def joint_name(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'joint_name' field must be of type 'str'"
        self._joint_name = value

    @builtins.property
    def joint_state_topic(self):
        """Message field 'joint_state_topic'."""
        return self._joint_state_topic

    @joint_state_topic.setter
    def joint_state_topic(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'joint_state_topic' field must be of type 'str'"
        self._joint_state_topic = value

    @builtins.property
    def pressed_topic(self):
        """Message field 'pressed_topic'."""
        return self._pressed_topic

    @pressed_topic.setter
    def pressed_topic(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'pressed_topic' field must be of type 'str'"
        self._pressed_topic = value

    @builtins.property
    def state_topic(self):
        """Message field 'state_topic'."""
        return self._state_topic

    @state_topic.setter
    def state_topic(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'state_topic' field must be of type 'str'"
        self._state_topic = value

    @builtins.property
    def supported_commands(self):
        """Message field 'supported_commands'."""
        return self._supported_commands

    @supported_commands.setter
    def supported_commands(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'supported_commands' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'supported_commands' field must be an unsigned integer in [0, 255]"
        self._supported_commands = value

    @builtins.property
    def unit(self):
        """Message field 'unit'."""
        return self._unit

    @unit.setter
    def unit(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'unit' field must be of type 'str'"
        self._unit = value

    @builtins.property
    def min_position(self):
        """Message field 'min_position'."""
        return self._min_position

    @min_position.setter
    def min_position(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'min_position' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'min_position' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._min_position = value

    @builtins.property
    def max_position(self):
        """Message field 'max_position'."""
        return self._max_position

    @max_position.setter
    def max_position(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'max_position' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'max_position' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._max_position = value

    @builtins.property
    def state_ids(self):
        """Message field 'state_ids'."""
        return self._state_ids

    @state_ids.setter
    def state_ids(self, value):
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
                 all(isinstance(v, str) for v in value) and
                 True), \
                "The 'state_ids' field must be a set or sequence and each value of type 'str'"
        self._state_ids = value

    @builtins.property
    def state_labels(self):
        """Message field 'state_labels'."""
        return self._state_labels

    @state_labels.setter
    def state_labels(self, value):
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
                 all(isinstance(v, str) for v in value) and
                 True), \
                "The 'state_labels' field must be a set or sequence and each value of type 'str'"
        self._state_labels = value

    @builtins.property
    def state_positions(self):
        """Message field 'state_positions'."""
        return self._state_positions

    @state_positions.setter
    def state_positions(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'd', \
                "The 'state_positions' array.array() must have the type code of 'd'"
            self._state_positions = value
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
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -1.7976931348623157e+308 or val > 1.7976931348623157e+308) or math.isinf(val) for val in value)), \
                "The 'state_positions' field must be a set or sequence and each value of type 'float' and each double in [-179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000, 179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000]"
        self._state_positions = array.array('d', value)

    @builtins.property
    def requires_grasp(self):
        """Message field 'requires_grasp'."""
        return self._requires_grasp

    @requires_grasp.setter
    def requires_grasp(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'requires_grasp' field must be of type 'bool'"
        self._requires_grasp = value

    @builtins.property
    def operable(self):
        """Message field 'operable'."""
        return self._operable

    @operable.setter
    def operable(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'operable' field must be of type 'bool'"
        self._operable = value

    @builtins.property
    def unavailable_reason(self):
        """Message field 'unavailable_reason'."""
        return self._unavailable_reason

    @unavailable_reason.setter
    def unavailable_reason(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'unavailable_reason' field must be of type 'str'"
        self._unavailable_reason = value

    @builtins.property
    def required_toolset(self):
        """Message field 'required_toolset'."""
        return self._required_toolset

    @required_toolset.setter
    def required_toolset(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'required_toolset' field must be of type 'str'"
        self._required_toolset = value

    @builtins.property
    def toolset_compatible(self):
        """Message field 'toolset_compatible'."""
        return self._toolset_compatible

    @toolset_compatible.setter
    def toolset_compatible(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'toolset_compatible' field must be of type 'bool'"
        self._toolset_compatible = value

    @builtins.property
    def adapter_validated(self):
        """Message field 'adapter_validated'."""
        return self._adapter_validated

    @adapter_validated.setter
    def adapter_validated(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'adapter_validated' field must be of type 'bool'"
        self._adapter_validated = value

    @builtins.property
    def default_force(self):
        """Message field 'default_force'."""
        return self._default_force

    @default_force.setter
    def default_force(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'default_force' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'default_force' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._default_force = value

    @builtins.property
    def min_trigger_force(self):
        """Message field 'min_trigger_force'."""
        return self._min_trigger_force

    @min_trigger_force.setter
    def min_trigger_force(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'min_trigger_force' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'min_trigger_force' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._min_trigger_force = value

    @builtins.property
    def max_force(self):
        """Message field 'max_force'."""
        return self._max_force

    @max_force.setter
    def max_force(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'max_force' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'max_force' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._max_force = value

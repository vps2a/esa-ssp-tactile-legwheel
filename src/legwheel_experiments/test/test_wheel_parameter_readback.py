"""Exercise integer ROS type codes without requiring a ROS installation."""

from dataclasses import dataclass
from enum import Enum
from types import SimpleNamespace

import pytest

from legwheel_experiments.wheel_control import validate_wheel_parameter_readback


# Jazzy uses Enum, not IntEnum. These values are the ParameterType.msg codes.
class ParameterType(Enum):
    DOUBLE = 3
    STRING = 4


@dataclass(frozen=True)
class Parameter:
    name: str
    value: str | float

    def get_parameter_value(self):
        is_string = isinstance(self.value, str)
        return SimpleNamespace(
            type=(ParameterType.STRING if is_string else ParameterType.DOUBLE).value,
            string_value=self.value if is_string else "",
            double_value=0.0 if is_string else self.value,
        )


@pytest.fixture(params=["raw_torque", "raw_torque_velocity_capped"])
def parameters(request):
    return [
        Parameter("wheel_control_mode", request.param),
        Parameter("wheel_speed_cap_rad_s", 0.0 if request.param == "raw_torque" else 3.0),
        Parameter("wheel_governor_kp", 0.5),
        Parameter("wheel_governor_ki", 0.0),
        Parameter("wheel_control_session_id", "example_run"),
    ]


def test_matching_wire_values_pass_even_though_codes_are_not_enum_members(parameters):
    reported = [parameter.get_parameter_value() for parameter in parameters]
    assert reported[0].type == 4
    assert reported[0].type != ParameterType.STRING
    assert reported[1].type == 3
    assert reported[1].type != ParameterType.DOUBLE
    validate_wheel_parameter_readback(parameters, reported)


@pytest.mark.parametrize("index,field,value", [
    (0, "string_value", "wrong_mode"),
    (1, "double_value", 4.0),
    (2, "double_value", 0.75),
    (3, "double_value", 0.1),
    (4, "string_value", "different_run"),
])
def test_real_value_mismatch_still_fails_and_identifies_the_parameter(parameters, index, field, value):
    reported = [parameter.get_parameter_value() for parameter in parameters]
    setattr(reported[index], field, value)
    with pytest.raises(RuntimeError, match=parameters[index].name):
        validate_wheel_parameter_readback(parameters, reported)


def test_wrong_type_is_rejected_even_when_inactive_value_field_matches(parameters):
    reported = [parameter.get_parameter_value() for parameter in parameters]
    reported[3].type = 0  # NOT_SET also carries a default double_value of zero.
    with pytest.raises(RuntimeError, match="wheel_governor_ki: expected ROS type 3, received 0"):
        validate_wheel_parameter_readback(parameters, reported)


@pytest.mark.parametrize("extra", [False, True])
def test_missing_or_extra_values_are_rejected(parameters, extra):
    reported = [parameter.get_parameter_value() for parameter in parameters]
    reported = reported + [reported[0]] if extra else reported[:-1]
    with pytest.raises(RuntimeError, match="expected 5 parameters"):
        validate_wheel_parameter_readback(parameters, reported)

from typing import Callable

import wpilib
from wpilib import Preferences
from wpilib.interfaces import MotorController
import phoenix6


def clamp(value: float, min_value: float, max_value: float) -> float:
    """Restrict value between min_value and max_value."""
    return max(min(value, max_value), min_value)


def curve(
    mapping: Callable[[float], float],
    offset: float,
    deadband: float,
    max_mag: float,
    absolute_offset: bool = True,
) -> Callable[[float], float]:
    """Return a function that applies a curve to an input.

    Arguments:
    mapping -- maps input to output
    offset -- added to output, even if the input is deadbanded
    deadband -- when the input magnitude is less than this,
        the input is treated as zero
    max_mag -- restricts the output magnitude to a maximum.
        If this is 0, no restriction is applied.
    absolute_offset -- If true, applies offset always (even when deadbanded),
        If false, adds sign(input_val) * offset or 0 in the deadband
    """

    def f(input_val: float) -> float:
        """Apply a curve to an input. Be sure to call this function to get an output, not curve."""
        if abs(input_val) < deadband:
            return offset if absolute_offset else 0
        applied_offset = (1 if absolute_offset else abs(input_val) / input_val) * offset
        output_val = mapping(input_val) + applied_offset
        if max_mag == 0:
            return output_val
        else:
            return clamp(output_val, -max_mag, max_mag)

    return f


def linear_curve(
    scalar: float = 1.0,
    offset: float = 0.0,
    deadband: float = 0.0,
    max_mag: float = 0.0,
    absolute_offset: bool = True,
) -> Callable[[float], float]:
    return curve(lambda x: scalar * x, offset, deadband, max_mag, absolute_offset)


def ollie_curve(
    scalar: float = 1.0,
    offset: float = 0.0,
    deadband: float = 0.0,
    max_mag: float = 0.0,
    absolute_offset: bool = True,
) -> Callable[[float], float]:
    return curve(
        lambda x: scalar * x * abs(x), offset, deadband, max_mag, absolute_offset
    )


def cubic_curve(
    scalar: float = 1.0,
    offset: float = 0.0,
    deadband: float = 0.0,
    max_mag: float = 0.0,
    absolute_offset: bool = True,
) -> Callable[[float], float]:
    return curve(lambda x: scalar * x**3, offset, deadband, max_mag, absolute_offset)


class SmartPreference(object):
    """Wrapper for wpilib Preferences that improves it in three ways:
    1. Previous values from NetworkTables are remembered if connection
    is lost instead of defaulting to the values set in code
    2. Everything is done dynamically so there is no need to specify
    type. However, because of NT limitations, the type must stay the
    same throughout the entirety of the code
    3. Initializing, getting, and setting Preferences is made much
    easier and enables this class to be a drop-in replacement for normal
    values. For example:
    ```
    class MyComponent:

        # initialize a preference with NT key "foo" and default value True
        # SmartPreferences MUST be class attributes (ie. initialized under the header)
        # Values must be of type int, float, str, or bool
        foo = SmartPreference(True)

        def execute(self):

            # retrieve the preference from NT (defaults to previous value)
            foo = self.foo

            # set the preference in NT
            self.foo = False
    ```
    """

    def __init__(self, value) -> None:
        self._value = value
        self._type = type(value)
        if self._type not in (int, float, str, bool):
            raise TypeError(
                f"SmartPreference must be int, float, str, or bool (not {self._type})"
            )

    def __set_name__(self, obj, name):
        self._key = name
        if self._type == int or self._type == float:
            Preferences.initDouble(self._key, self._value)
        elif self._type == str:
            Preferences.initString(self._key, self._value)
        elif self._type == bool:
            Preferences.initBoolean(self._key, self._value)

    def __get__(self, obj, objtype=None):
        if self._type == int or self._type == float:
            self._value = Preferences.getDouble(self._key, self._value)
        elif self._type == str:
            self._value = Preferences.getString(self._key, self._value)
        elif self._type == bool:
            self._value = Preferences.getBoolean(self._key, self._value)
        return self._value

    def __set__(self, obj, value):
        if type(value) != self._type:
            raise TypeError(
                f"Set value type ({type(value)} does not match original ({self._type}))"
            )
        self._value = value
        self._type = type(value)
        if self._type == int or self._type == float:
            self._value = Preferences.setDouble(self._key, self._value)
        elif self._type == str:
            self._value = Preferences.setString(self._key, self._value)
        elif self._type == bool:
            self._value = Preferences.setBoolean(self._key, self._value)


class WPI_TalonFX(phoenix6.hardware.TalonFX, MotorController):
    """Wrapper for the phoenix6 TalonFX that implements
    the wpilib MotorController interface, making it possible
    to use TalonFX controllers in, for example, MotorControllerGroup
    and DifferentialDrive
    """

    def __init__(self, device_id: int, canbus: str = "", enable_foc: bool = False):
        phoenix6.hardware.TalonFX.__init__(self, device_id, canbus=canbus)
        MotorController.__init__(self)
        self.config = phoenix6.configs.TalonFXConfiguration()
        self.duty_cycle_out = phoenix6.controls.DutyCycleOut(0, enable_foc=enable_foc)
        self.voltage_out = phoenix6.controls.VoltageOut(0, enable_foc=enable_foc)
        self.is_disabled = False

    def disable(self):
        self.stopMotor()
        self.is_disabled = True

    def get(self) -> float:
        return self.duty_cycle_out.output

    def getInverted(self) -> bool:
        return (
            self.config.motor_output.inverted
            == phoenix6.signals.InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        )

    def set(self, speed: float):
        if not self.is_disabled:
            self.duty_cycle_out.output = speed
            self.set_control(self.duty_cycle_out)

    def setIdleMode(self, mode: phoenix6.signals.NeutralModeValue):
        """Set the idle mode setting

        Arguments:
        mode -- Idle mode (coast or brake)
        """
        self.config.motor_output.neutral_mode = mode
        self.configurator.apply(self.config)

    def setInverted(self, isInverted: bool):
        if isInverted:
            self.config.motor_output.inverted = (
                phoenix6.signals.InvertedValue.CLOCKWISE_POSITIVE
            )
        else:
            self.config.motor_output.inverted = (
                phoenix6.signals.InvertedValue.COUNTER_CLOCKWISE_POSITIVE
            )
        self.configurator.apply(self.config)

    def setVoltage(self, volts: float):
        if not self.is_disabled:
            self.voltage_out.output = volts
            self.set_control(self.voltage_out)

    def stopMotor(self):
        self.set(0)

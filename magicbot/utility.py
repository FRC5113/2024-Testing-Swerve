from typing import Callable

import wpilib
from wpilib import Preferences, SmartDashboard
from wpilib.interfaces import MotorController
from wpimath.trajectory import TrapezoidProfile
from wpimath.controller import ProfiledPIDController, SimpleMotorFeedforwardMeters
from wpimath.units import seconds
from wpiutil import Sendable, SendableBuilder
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


class SmartGain:
    """Used internally by SmartProfile and SmartController"""

    def __init__(self, key, value, updater):
        Preferences.initDouble(key, value)
        self.key = key
        self.value = Preferences.getDouble(key, value)
        self.updater = updater

    def set(self, value):
        if value != self.value:
            self.value = value
            Preferences.setDouble(self.key, self.value)

    def get(self):
        from_preferences = Preferences.getDouble(self.key, self.value)
        if self.value != from_preferences:
            self.value = from_preferences
        return self.value

    def update_controller(self, controller):
        self.updater(controller, self.value)


class SmartController(ProfiledPIDController, Sendable):
    """Wraps a `ProfiledPIDController` and a `SimpleMotorFeedforward`
    together and uses Preferences to allow for dynamic gain setting.
    This should **only** be created from the `create_controller()`
    method in `SmartProfile`
    """

    def __init__(self, gains, period=0.02) -> None:
        ProfiledPIDController.__init__(
            self, 0, 0, 0, TrapezoidProfile.Constraints(0, 0), period
        )
        self.feedforward = SimpleMotorFeedforwardMeters(0, 0, 0)
        Sendable.__init__(self)
        self._gains = gains
        for gain in self._gains:
            gain.update_controller(self)
        self._measurement = None
        self._output = 0

    def initSendable(self, builder: SendableBuilder) -> None:
        builder.setSmartDashboardType("SmartController")
        builder.addDoubleProperty(
            "Setpoint", lambda: self.getSetpoint().position, lambda _: None
        )
        builder.addDoubleProperty(
            "Goal", lambda: self.getGoal().position, lambda _: None
        )
        builder.addDoubleProperty(
            "Measurement", lambda: self.getMeasurement(), lambda _: None
        )
        builder.addDoubleProperty(
            "Error", lambda: self.getPositionError(), lambda _: None
        )
        builder.addDoubleProperty("Output", lambda: self.getOutput(), lambda _: None)

    def _update(self):
        for gain in self._gains:
            gain.update_controller(self)

    def calculate(self, measurement: float, goal: float = None) -> float:
        """Overridden. Get output based on a provided measurement and a
        goal.

        Args:
            measurement (float): measurement of the process variable
            goal (float, optional): goal position. Defaults to None.

        Returns:
            float: output from feedforward and profiledpid
        """
        self._measurement = measurement
        self._update()
        self._output = self.feedforward.calculate(self.getSetpoint().position)
        if goal is None:
            self._output += super().calculate(measurement)
        else:
            self.setGoal(goal)
            self._output += super().calculate(measurement)
        return self._output

    def getMeasurement(self) -> float:
        """Returns measurement most recently passed to `calculate()`"""
        if self._measurement is None:
            return 0
        return self._measurement

    def getOutput(self) -> float:
        """Returns most recent output from `calculate()`"""
        return self._output

    def setS(self, value: float) -> None:
        """Set feedfoward kS to `value`"""
        self.feedforward = SimpleMotorFeedforwardMeters(
            value, self.feedforward.kV, self.feedforward.kA
        )

    def setV(self, value: float) -> None:
        """Set feedfoward kV to `value`"""
        self.feedforward = SimpleMotorFeedforwardMeters(
            self.feedforward.kS, value, self.feedforward.kA
        )

    def setA(self, value: float) -> None:
        """Set feedfoward kA to `value`"""
        self.feedforward = SimpleMotorFeedforwardMeters(
            self.feedforward.kS, self.feedforward.kV, value
        )

    def setMaxV(self, value: float) -> None:
        """Set max velocity to `value`"""
        constraints = self.getConstraints()
        self.setConstraints(
            TrapezoidProfile.Constraints(value, constraints.maxAcceleration)
        )

    def setMaxA(self, value: float) -> None:
        """Set max acceleration to `value`"""
        constraints = self.getConstraints()
        self.setConstraints(
            TrapezoidProfile.Constraints(constraints.maxVelocity, value)
        )


class SmartProfile(Sendable):
    """Stores several gains that are commonly used for control. This
    class allows the use of NetworkTables to change the gains on the
    fly, and uses Preferences so that the gains are stored locally on
    the robot. Use the `create_controller()` method to create a
    `SmartController` with synchronized gains.
    """

    def __init__(
        self,
        key: str,
        kP=0.0,
        kI=0.0,
        kD=0.0,
        kS=0.0,
        kV=0.0,
        kA=0.0,
        kMaxV=0.0,
        kMaxA=0.0,
        continuous_range: tuple[float] = None,
    ) -> None:
        """Create a SmartProfile with the designated gains. There will
        only be a trapezoidal profile if `kMaxV` and `kMaxA` are set.
        The input will only be continuous if `continuous_range` is set.

        Args:
            key (str): Prefix of gains in SmartDashboard
            kP (float, optional): Proportional term. Defaults to 0.0.
            kI (float, optional): Integral term. Defaults to 0.0.
            kD (float, optional): Derivative term. Defaults to 0.0.
            kS (float, optional): Static voltage. Defaults to 0.0.
            kV (float, optional): Cruise voltage. Defaults to 0.0.
            kA (float, optional): Acceleration voltage. Defaults to 0.0.
            kMaxV (float, optional): Maximum velocity. Defaults to 0.0.
            kMaxA (float, optional): Maximum acceleration. Defaults to 0.0.
            continuous_range (tuple[float], optional): Tuple containing
                minimum and maximum value. Use to specify continuous
                input. Defaults to None.
        """
        Sendable.__init__(self)
        self._gains = (
            SmartGain(
                f"{key}_kP", kP, (lambda controller, value: controller.setP(value))
            ),
            SmartGain(
                f"{key}_kI", kI, (lambda controller, value: controller.setI(value))
            ),
            SmartGain(
                f"{key}_kD", kD, (lambda controller, value: controller.setD(value))
            ),
            SmartGain(
                f"{key}_kS", kS, (lambda controller, value: controller.setS(value))
            ),
            SmartGain(
                f"{key}_kV", kV, (lambda controller, value: controller.setV(value))
            ),
            SmartGain(
                f"{key}_kA", kA, (lambda controller, value: controller.setA(value))
            ),
            SmartGain(
                f"{key}_kMaxV",
                kMaxV,
                (lambda controller, value: controller.setMaxV(value)),
            ),
            SmartGain(
                f"{key}_kMaxA",
                kMaxA,
                (lambda controller, value: controller.setMaxA(value)),
            ),
        )
        self.continuous_range = continuous_range

    def initSendable(self, builder: SendableBuilder) -> None:
        builder.setSmartDashboardType("SmartProfile")
        for i in range(len(self._gains)):
            builder.addDoubleProperty(
                self._gains[i].key, self._gains[i].get, self._create_setter(i)
            )

    def _create_setter(self, index):
        # used to avoid late binding
        return lambda x: self._gains[index].set(x)

    def create_controller(self, period=0.02) -> SmartController:
        """Creates new `SmartController` with synchronized gains. This
        should be the only way that SmartControllers are created.

        Args:
            period (float, optional): Delta time. Defaults to 0.02.

        Returns:
            SmartController: Created SmartController
        """
        controller = SmartController(self._gains, period)
        if self.continuous_range is not None:
            controller.enableContinuousInput(
                self.continuous_range[0], self.continuous_range[1]
            )
        return controller


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

    _changed_flag = False

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
        new = None
        if self._type == int or self._type == float:
            new = Preferences.getDouble(self._key, self._value)
        elif self._type == str:
            new = Preferences.getString(self._key, self._value)
        elif self._type == bool:
            new = Preferences.getBoolean(self._key, self._value)
        if new != self._value:
            SmartPreference._changed_flag = True
            self._value = new
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

    def has_changed() -> bool:
        """Returns if any SmartPreference has changed since checked.
        Only works if called statically."""
        if SmartPreference._changed_flag:
            SmartPreference._changed_flag = False
            return True
        return False


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

from phoenix6.hardware import CANcoder, TalonFX
import math
import wpilib
from phoenix6.signals import NeutralModeValue, FeedbackSensorSourceValue
from wpimath.kinematics import SwerveModuleState
from wpimath.geometry import Rotation2d

from phoenix6 import configs, controls


class SwerveWheel:
    drive_gear_ratio: float
    wheel_radius: float
    speed_motor: TalonFX
    speed_configs: configs.TalonFXConfiguration
    direction_motor: TalonFX
    direction_configs: configs.TalonFXConfiguration
    cancoder: CANcoder
    debug: bool

    def setup(self) -> None:
        """
        This function is automatically called after the motors and encoders have been injected.
        """

        # initialize to brake mode
        self.direction_configs.motor_output.neutral_mode = NeutralModeValue.BRAKE
        self.speed_configs.motor_output.neutral_mode = NeutralModeValue.BRAKE
        # set cancoder as remote sensor
        self.direction_configs.feedback.feedback_remote_sensor_id = (
            self.cancoder.device_id
        )
        self.direction_configs.feedback.feedback_sensor_source = (
            FeedbackSensorSourceValue.REMOTE_CANCODER
        )
        # apply configs
        self.direction_motor.configurator.apply(self.direction_configs)
        self.speed_motor.configurator.apply(self.speed_configs)

        self.desiredState = None
        self.stopped = False
        # create a Motion Magic Expo Voltage request for direction motor
        self.direction_request = controls.MotionMagicExpoVoltage(0)
        # create a Motion Magic Velocity Voltage request for speed motor
        self.speed_request = controls.MotionMagicVelocityVoltage(0)

    """
    CONTROL METHODS
    """

    def setDesiredState(self, state: SwerveModuleState):
        self.desiredState = state

    def stopWheel(self) -> None:
        # consider using another type of brake/stop
        self.speed_motor.set_control(controls.static_brake.StaticBrake())
        self.direction_motor.set_control(controls.coast_out.CoastOut())

        # Prevents SmartDashboard desync
        if self.debug:
            wpilib.SmartDashboard.putNumber(str(self.speed_motor.device_id) + " Mag", 0)

        self.stopped = True

    def getDirectionMotorPos(self) -> None:
        return self.direction_motor.get_position().value / (150 / 7)

    """
    EXECUTE
    """

    def execute(self) -> None:
        if (
            self.stopped or self.desiredState is None
        ):  # Stops angle from updating when stopped.
            self.stopped = False
            return

        encoder_rotation = Rotation2d(self.cancoder.get_position().value * 2 * math.pi)
        state = SwerveModuleState.optimize(self.desiredState, encoder_rotation)
        # scale speed while turning
        state.speed *= (state.angle - encoder_rotation).cos()
        # convert speed from m/s to r/s
        state.speed *= self.drive_gear_ratio / (self.wheel_radius * 2 * math.pi)
        self.direction_motor.set_control(
            self.direction_request.with_position(state.angle.radians / 2 / math.pi)
        )
        self.speed_motor.set_control(self.speed_request.with_velocity(state.speed))

        if self.debug:
            wpilib.SmartDashboard.putNumber(
                str(self.direction_motor.device_id) + "angle_r (degrees)",
                state.angle.degrees(),
            )
            wpilib.SmartDashboard.putNumber(
                str(self.direction_motor.device_id) + "angle_e (degrees)",
                state.angle.degrees()
                - self.cancoder.get_absolute_position().value * 360,
            )
            wpilib.SmartDashboard.putNumber(
                str(self.direction_motor.device_id) + "angle_y (degrees)",
                self.cancoder.get_absolute_position().value * 360,
            )
            wpilib.SmartDashboard.putNumber(
                str(self.direction_motor.device_id) + "angle_u (volts)",
                self.direction_motor.get_motor_voltage(),
            )
            wpilib.SmartDashboard.putNumber(
                str(self.speed_motor.device_id) + "speed_r (rps)", state.speed
            )
            wpilib.SmartDashboard.putNumber(
                str(self.direction_motor.device_id) + "speed_e (rps)",
                state.speed - self.speed_motor.get_velocity().value,
            )
            wpilib.SmartDashboard.putNumber(
                str(self.direction_motor.device_id) + "speed_y (rps)",
                self.speed_motor.get_velocity().value,
            )
            wpilib.SmartDashboard.putNumber(
                str(self.direction_motor.device_id) + "speed_u (volts)",
                self.speed_motor.get_motor_voltage(),
            )

from phoenix6.hardware import CANcoder, TalonFX
import math
import wpilib
from phoenix6.signals import NeutralModeValue, FeedbackSensorSourceValue
from wpimath.kinematics import SwerveModuleState, SwerveModulePosition
from wpimath.geometry import Rotation2d
from phoenix6 import configs, controls
from magicbot import will_reset_to


class SwerveWheel:
    drive_gear_ratio: float
    wheel_radius: float
    speed_motor: TalonFX
    speed_configs: configs.TalonFXConfiguration
    direction_motor: TalonFX
    direction_configs: configs.TalonFXConfiguration
    cancoder: CANcoder
    debug: bool

    """Module must be explicitly told to move (via setDesiredState) each
    loop, otherwise it defaults to stopped for safety.
    """
    stopped = will_reset_to(True)
    update = will_reset_to(False)

    def setup(self) -> None:
        """
        This function is automatically called after the motors and encoders have been injected.
        """

        # apply configs
        self.init_configs()
        self.direction_motor.configurator.apply(self.direction_configs)
        self.speed_motor.configurator.apply(self.speed_configs)

        self.desired_state = None
        self.direction_request = controls.MotionMagicVoltage(0)
        self.speed_request = controls.MotionMagicVelocityVoltage(0)

    def init_configs(self):
        # initialize to brake mode
        self.direction_configs.motor_output.neutral_mode = NeutralModeValue.BRAKE
        self.speed_configs.motor_output.neutral_mode = NeutralModeValue.BRAKE
        # FIX CANCODER!!!! (use feedback rotor offset?)
        # set cancoder as remote sensor
        # self.direction_configs.feedback.feedback_sensor_source = (
        #     FeedbackSensorSourceValue.REMOTE_CANCODER
        # )
        # self.direction_configs.feedback.feedback_remote_sensor_id = (
        #     self.cancoder.device_id
        # )

    """
    CONTROL METHODS
    """

    def setDesiredState(self, state: SwerveModuleState):
        self.stopped = False
        self.desired_state = state

    def update_configs(self, direction_configs, speed_configs):
        self.direction_configs = direction_configs
        self.speed_configs = speed_configs
        self.init_configs()
        self.update = True

    """
    EXECUTE
    """

    def execute(self) -> None:
        # update configs if change detected
        if self.update:
            self.direction_motor.configurator.apply(self.direction_configs)
            self.speed_motor.configurator.apply(self.speed_configs)

        if self.stopped:
            self.speed_motor.set_control(controls.static_brake.StaticBrake())
            self.direction_motor.set_control(controls.coast_out.CoastOut())
            # Prevents SmartDashboard desync
            if self.debug:
                wpilib.SmartDashboard.putNumber(
                    str(self.speed_motor.device_id) + " Mag", 0
                )
            return

        encoder_rotation = Rotation2d(self.cancoder.get_position().value * 2 * math.pi)
        state = SwerveModuleState.optimize(self.desired_state, encoder_rotation)
        # scale speed while turning
        state.speed *= (state.angle - encoder_rotation).cos()
        # convert speed from m/s to r/s
        state.speed *= self.drive_gear_ratio / (self.wheel_radius * 2 * math.pi)
        self.direction_motor.set_control(
            self.direction_request.with_position(state.angle.radians() / 2 / math.pi)
        )
        self.speed_motor.set_control(self.speed_request.with_velocity(state.speed))

        if self.debug:
            # if self.cancoder.device_id == 13:
                # print(
                #     self.direction_motor.get_closed_loop_reference().value,
                #     self.direction_motor.get_closed_loop_reference().value
                #     - self.direction_motor.get_closed_loop_error().value,
                #     self.direction_motor.get_motor_voltage().value,
                # )
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
                self.direction_motor.get_motor_voltage().value,
            )
            wpilib.SmartDashboard.putNumber(
                str(self.speed_motor.device_id) + "speed_r (rps)", 
                self.speed_motor.get_closed_loop_reference().value
            )
            wpilib.SmartDashboard.putNumber(
                str(self.direction_motor.device_id) + "speed_e (rps)",
                self.speed_motor.get_closed_loop_error().value,
            )
            wpilib.SmartDashboard.putNumber(
                str(self.direction_motor.device_id) + "speed_y (rps)",
                self.speed_motor.get_rotor_velocity().value
            )
            wpilib.SmartDashboard.putNumber(
                str(self.direction_motor.device_id) + "speed_u (volts)",
                self.speed_motor.get_motor_voltage().value,
            )

from phoenix6.hardware import CANcoder, TalonFX
import math
from wpilib import SmartDashboard
from phoenix6.signals import NeutralModeValue
from phoenix6.configs import TalonFXConfiguration
from wpimath.kinematics import SwerveModuleState, SwerveModulePosition
from wpimath.controller import HolonomicDriveController, PIDController, ProfiledPIDControllerRadians
from wpimath.trajectory import TrapezoidProfileRadians
from wpimath.geometry import Rotation2d
from phoenix6 import controls
from magicbot import will_reset_to

from util.smart_preference import SmartProfile


class SwerveWheel:
    drive_gear_ratio: float
    wheel_radius: float
    max_speed: float
    speed_motor: TalonFX
    speed_profile: SmartProfile
    direction_motor: TalonFX
    direction_profile: SmartProfile
    cancoder: CANcoder

    """Module must be explicitly told to move (via setDesiredState) each
    loop, otherwise it defaults to stopped for safety.
    """
    stopped = will_reset_to(True)

    def setup(self) -> None:
        """
        This function is automatically called after the motors and encoders have been injected.
        """

        # apply configs
        self.motor_configs = TalonFXConfiguration()
        self.motor_configs.motor_output.neutral_mode = NeutralModeValue.COAST
        self.direction_motor.configurator.apply(self.motor_configs)
        self.speed_motor.configurator.apply(self.motor_configs)

        self.speed_controller = self.speed_profile.create_controller(
            f"{self.speed_motor.device_id}_speed"
        )
        self.direction_controller = self.direction_profile.create_controller(
            f"{self.direction_motor.device_id}_direction"
        )

        self.desired_state = None

        # Initialize controllers for trajectory following
        self.x_controller = PIDController(1.0, 0.0, 0.0)
        self.y_controller = PIDController(1.0, 0.0, 0.0)
        self.theta_controller = ProfiledPIDControllerRadians(
            1.0, 0.0, 0.0,
            TrapezoidProfileRadians.Constraints(self.max_speed * 2 * math.pi, 3.14 )
        )
        self.holonomic_controller = HolonomicDriveController(self.x_controller, self.y_controller, self.theta_controller)

    """
    INFORMATIONAL METHODS
    """

    def getMeasuredState(self):
        """Retrieve list of measured angle and velocity
        (used for AdvantageScope)
        """
        if self.stopped:
            return [0, 0]
        return [
            self.cancoder.get_absolute_position().value * 360,
            self.speed_motor.get_velocity().value
            * (self.wheel_radius * 2 * math.pi)
            / self.drive_gear_ratio,
        ]

    def getPosition(self) -> SwerveModulePosition:
        return SwerveModulePosition(
            self.speed_motor.get_position().value
            / self.drive_gear_ratio
            * (self.wheel_radius * 2 * math.pi),
            Rotation2d(
                self.cancoder.get_absolute_position().value * math.tau + math.pi / 2
            ),
        )

    """
    CONTROL METHODS
    """

    def setDesiredState(self, state: SwerveModuleState):
        self.stopped = False
        self.desired_state = state

    """
    EXECUTE
    """

    def execute(self) -> None:
        if self.stopped:
            self.speed_motor.set_control(controls.static_brake.StaticBrake())
            self.direction_motor.set_control(controls.coast_out.CoastOut())
            return

        encoder_rotation = Rotation2d(
            self.cancoder.get_absolute_position().value * 2 * math.pi
        )
        state = SwerveModuleState.optimize(self.desired_state, encoder_rotation)
        # scale speed while turning
        state.speed *= (state.angle - encoder_rotation).cos()
        # convert speed from m/s to r/s
        state.speed *= self.drive_gear_ratio / (self.wheel_radius * 2 * math.pi)
        speed_output = self.speed_controller.calculate(
            self.speed_motor.get_velocity().value, state.speed
        )
        self.speed_motor.set_control(controls.VoltageOut(speed_output))
        direction_output = self.direction_controller.calculate(
            encoder_rotation.radians(),
            state.angle.radians(),
        )
        self.direction_motor.set_control(controls.VoltageOut(-direction_output))

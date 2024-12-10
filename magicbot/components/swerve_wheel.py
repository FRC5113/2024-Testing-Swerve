import math

from phoenix6 import controls
from phoenix6.configs import TalonFXConfiguration
from phoenix6.hardware import CANcoder, TalonFX
from phoenix6.signals import NeutralModeValue
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModulePosition, SwerveModuleState
from wpimath import applyDeadband

from magicbot import will_reset_to
from util.smart_preference import SmartProfile, SmartPreference


class SwerveWheel:
    drive_gear_ratio: float
    wheel_radius: float
    speed_motor: TalonFX
    speed_profile: SmartProfile
    direction_motor: TalonFX
    direction_profile: SmartProfile
    cancoder: CANcoder

    """Module must be explicitly told to move (via setDesiredState) each
    loop, otherwise it defaults to stopped for safety.
    """
    stopped = will_reset_to(True)
    angle_deadband = SmartPreference(0.0349) 
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

        
      

    """
    INFORMATIONAL METHODS
    """

    def getMeasuredState(self):
        """Retrieve list of measured angle and velocity
        (used for AdvantageScope)
        """
        
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
            Rotation2d(self.cancoder.get_absolute_position().value * math.tau),
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

        angle_error = state.angle - encoder_rotation

        deadbanded_error = applyDeadband(angle_error.radians(), self.angle_deadband)

        if deadbanded_error == 0:
            self.direction_motor.set_control(controls.coast_out.CoastOut())
        else:
            direction_output = self.direction_controller.calculate(
                encoder_rotation.radians(),
                state.angle.radians(),
            )
            self.direction_motor.set_control(controls.VoltageOut(-direction_output))

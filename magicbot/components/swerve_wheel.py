from constants import *
from phoenix6.hardware import CANcoder, TalonFX
from math import fabs
import wpilib
from phoenix6.signals import NeutralModeValue, FeedbackSensorSourceValue
from phoenix6 import configs, controls


class SwerveWheel:
    speed_motor: TalonFX
    direction_motor: TalonFX
    cancoder: CANcoder

    def setup(self) -> None:
        """
        This function is automatically called after the motors and encoders have been injected.
        """
        self.direction_talonfx_configs = configs.TalonFXConfiguration()
        self.speed_talonfx_configs = configs.TalonFXConfiguration()

        self.direction_talonfx_configs.motor_output.neutral_mode = NeutralModeValue.BRAKE
        self.speed_talonfx_configs.motor_output.neutral_mode = NeutralModeValue.BRAKE

        # set slot 0 gains
        slot0_configs = self.direction_talonfx_configs.slot0
        slot0_configs.k_s = 0.25  # Add 0.25 V output to overcome static friction
        slot0_configs.k_v = 0.12  # A velocity target of 1 rps results in 0.12 V output
        slot0_configs.k_a = 0.01  # An acceleration of 1 rps/s requires 0.01 V output
        slot0_configs.k_p = 4.8  # A position error of 2.5 rotations results in 12 V output
        slot0_configs.k_i = 0  # no output for integrated error
        slot0_configs.k_d = 0.1  # A velocity error of 1 rps results in 0.1 V output

        # set Motion Magic settings
        direction_motion_magic_configs = self.direction_talonfx_configs.motion_magic
        direction_motion_magic_configs.motion_magic_cruise_velocity = 80  # Target cruise velocity of 80 rps
        direction_motion_magic_configs.motion_magic_acceleration = 160  # Target acceleration of 160 rps/s (0.5 seconds)
        direction_motion_magic_configs.motion_magic_jerk = 1600  # Target jerk of 1600 rps/s/s (0.1 seconds)

        # set slot 1 gains
        slot1_configs = self.speed_talonfx_configs.slot0
        slot1_configs.k_s = 0.25  # Add 0.25 V output to overcome static friction
        slot1_configs.k_v = 0.12  # A velocity target of 1 rps results in 0.12 V output
        slot1_configs.k_a = 0.01  # An acceleration of 1 rps/s requires 0.01 V output
        slot1_configs.k_p = 4.8  # A position error of 2.5 rotations results in 12 V output
        slot1_configs.k_i = 0  # no output for integrated error
        slot1_configs.k_d = 0.1  # A velocity error of 1 rps results in 0.1 V output

        # set Motion Magic Expo settings
        direction_motion_magic_configs.motion_magic_cruise_velocity = 0  # Unlimited cruise velocity
        direction_motion_magic_configs.motion_magic_expo_k_v = 0.12  # kV is around 0.12 V/rps
        direction_motion_magic_configs.motion_magic_expo_k_a = 0.1  # Use a slower kA of 0.1 V/(rps/s)

        speed_motion_magic_configs = self.speed_talonfx_configs.motion_magic
        speed_motion_magic_configs.motion_magic_cruise_velocity = 0  # Unlimited cruise velocity
        speed_motion_magic_configs.motion_magic_expo_k_v = 0.12  # kV is around 0.12 V/rps
        speed_motion_magic_configs.motion_magic_expo_k_a = 0.1  # Use a slower kA of 0.1 V/(rps/s)

        self.direction_talonfx_configs.feedback.feedback_remote_sensor_id = self.cancoder.device_id
        self.direction_talonfx_configs.feedback.feedback_sensor_source = FeedbackSensorSourceValue.REMOTE_CANCODER

        self.direction_motor.configurator.apply(self.direction_talonfx_configs)
        self.speed_motor.configurator.apply(self.speed_talonfx_configs)

        self.cancoder.configSensorDirection(True, ktimeoutMs)

        self.directionTargetPos = 0.0
        self.directionTargetAngle = 0.0
        self.isInverted = False
        self.desiredAngle = 0
        self.desiredSpeed = 0
        self.stopped = False
        self.request = controls.MotionMagicExpoVoltage(0)

    def setDesiredAngle(self, angle: int) -> None:
        """
        Sets the desired angle we want the direction motor to turn to
        when the execute command is ran.
        """
        self.desiredAngle = angle

    def setDesiredSpeed(self, speed: int) -> None:
        self.desiredSpeed = max(-1, min(1, speed))

    def stopWheel(self) -> None:
        self.speed_motor.set_control(self.request.with_position(0))
        self.direction_motor.set_control(self.request.with_position(0))
        self.direction_talonfx_configs.motor_output.neutral_mode = NeutralModeValue.COAST

        if kDebug:
            wpilib.SmartDashboard.putNumber(
                str(self.speed_motor.device_id()) + " Mag", 0
            )

        self.stopped = True

    def getDirectionMotorPos(self) -> float:
        return self.speed_motor.get_position() / ksteeringGearRatio

    def execute(self) -> None:
        if self.stopped:
            self.stopped = False
            return

        self.desiredAngle %= 360

        angleDist = fabs(self.desiredAngle - self.directionTargetAngle)

        if 90 < angleDist < 270:
            targetAngle = (self.desiredAngle + 180) % 360
            self.isInverted = True
        else:
            targetAngle = self.desiredAngle
            self.isInverted = False

        targetAngleDist = fabs(targetAngle - self.directionTargetAngle)

        if targetAngleDist > 180:
            targetAngleDist = abs(targetAngleDist - 360)

        changeInTalonUnits = targetAngleDist / (360 / 2048)

        angleDiff = targetAngle - self.directionTargetAngle

        if angleDiff < 0:
            angleDiff += 360

        if angleDiff > 180:
            self.directionTargetPos -= changeInTalonUnits
        else:
            self.directionTargetPos += changeInTalonUnits

        self.directionTargetAngle = targetAngle

        if kDebug:
            wpilib.SmartDashboard.putNumber(
                str(self.speed_motor.device_id()) + " dirTargetAngle",
                self.directionTargetAngle,
            )
            wpilib.SmartDashboard.putNumber(
                str(self.speed_motor.device_id()) + " dirTargetPos",
                self.directionTargetPos,
            )
            wpilib.SmartDashboard.putBoolean(
                str(self.speed_motor.device_id()) + " Inverted?", self.isInverted
            )

        self.direction_motor.set(
            TalonFXControlMode.MotionMagic, self.directionTargetPos * ksteeringGearRatio
        )

        if self.isInverted:
            self.desiredSpeed *= -1

        angleDiff = self.directionTargetAngle - (
            self.direction_motor.getSelectedSensorPosition() / ksteeringGearRatio
        ) * (360 / 2048)
        angleDiff = (angleDiff + 180) % 360 - 180

        slowdownMult = max(-1.0, min(1.0, (-(3.14514 / 112006) * (angleDiff**2)) + 1))
        if not wpilib.RobotBase.isReal():
            slowdownMult = 1

        self.speed_motor.set(
            self.speed_motor.set_control(self.request.with_position()),
            max(-1, min(1, self.desiredSpeed * slowdownMult)),
        )

        if kDebug:
            wpilib.SmartDashboard.putNumber(
                str(self.speed_motor.getDeviceID()) + " Mag",
                max(-1, min(1, self.desiredSpeed * slowdownMult)),
            )
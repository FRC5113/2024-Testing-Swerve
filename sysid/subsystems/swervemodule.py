#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import math

import phoenix6
import wpimath.controller
import wpimath.geometry
import wpimath.kinematics
import wpimath.trajectory
from wpilib.interfaces import MotorController

kWheelRadius = 0.0508
kModuleMaxAngularVelocity = 30
kModuleMaxAngularAcceleration = 300


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


class SwerveModule:
    def __init__(
        self,
        driveMotorChannel: int,
        turningMotorChannel: int,
        turningEncoderChannel: int,
    ) -> None:
        """Constructs a SwerveModule with a drive motor, turning motor, and turning encoder.

        :param driveMotorChannel:      CAN ID for the drive motor.
        :param turningMotorChannel:    CAN ID for the turning motor.
        :param turningEncoderChannel:  CAN ID for the turning encoder
        """
        self.driveMotor = WPI_TalonFX(driveMotorChannel)
        self.turningMotor = WPI_TalonFX(turningMotorChannel)
        self.turningEncoder = phoenix6.hardware.CANcoder(turningEncoderChannel)

        # Gains are for example purposes only - must be determined for your own robot!
        self.drivePIDController = wpimath.controller.PIDController(0, 0, 0)

        # Gains are for example purposes only - must be determined for your own robot!
        self.turningPIDController = wpimath.controller.ProfiledPIDController(
            18.0,
            0,
            0,
            wpimath.trajectory.TrapezoidProfile.Constraints(
                kModuleMaxAngularVelocity,
                kModuleMaxAngularAcceleration,
            ),
        )

        # Gains are for example purposes only - must be determined for your own robot!
        self.driveFeedforward = wpimath.controller.SimpleMotorFeedforwardMeters(0, 0)
        self.turnFeedforward = wpimath.controller.SimpleMotorFeedforwardMeters(
            0.16, 0.375
        )
        self.voltage = 0

        # Limit the PID Controller's input range between -pi and pi and set the input
        # to be continuous.
        self.turningPIDController.enableContinuousInput(-math.pi, math.pi)

    def getState(self) -> wpimath.kinematics.SwerveModuleState:
        """Returns the current state of the module.

        :returns: The current state of the module.
        """
        return wpimath.kinematics.SwerveModuleState(
            self.driveMotor.get_velocity().value,
            wpimath.geometry.Rotation2d(
                self.turningEncoder.get_position().value * 2 * math.pi
            ),
        )

    def getPosition(self) -> wpimath.kinematics.SwerveModulePosition:
        """Returns the current position of the module.

        :returns: The current position of the module.
        """
        return wpimath.kinematics.SwerveModulePosition(
            self.driveMotor.get_velocity().value,
            wpimath.geometry.Rotation2d(
                self.turningEncoder.get_position().value * 2 * math.pi
            ),
        )

    def getDriveVoltage(self) -> float:
        return self.voltage

    def getDrivePosition(self) -> float:
        return self.driveMotor.get_position().value

    def getDriveVelocity(self) -> float:
        return self.driveMotor.get_velocity().value

    def setDesiredState(
        self, desiredState: wpimath.kinematics.SwerveModuleState
    ) -> None:
        """Sets the desired state for the module.

        :param desiredState: Desired state with speed and angle.
        """
        # treats speed as voltage for sysid purposes

        encoderRotation = wpimath.geometry.Rotation2d(
            self.turningEncoder.get_position().value * 2 * math.pi
        )

        # Optimize the reference state to avoid spinning further than 90 degrees
        state = wpimath.kinematics.SwerveModuleState.optimize(
            desiredState, encoderRotation
        )
        self.voltage = state.speed
        # state = desiredState

        # Calculate the turning motor output (in rad/s) from the turning PID controller.
        turnFeedback = self.turningPIDController.calculate(
            self.turningEncoder.get_position().value * 2 * math.pi,
            state.angle.radians(),
        )

        # convert the supplied velocity from the PID controller to a voltage with the feedforward
        turnOutput = self.turnFeedforward.calculate(turnFeedback)

        self.driveMotor.setVoltage(state.speed)
        self.turningMotor.setVoltage(-turnOutput)
        # puts pid stuff in terminal
        # if self.turningEncoder.device_id == 13:
        #     print(
        #         f"y: {encoderRotation.radians()}, r: {state.angle.radians()}, e: {encoderRotation.radians() - state.angle.radians()}, u: {turnFeedback}, ff: {turnOutput},"
        #     )

    def stop(self) -> None:
        self.driveMotor.setIdleMode(phoenix6.signals.NeutralModeValue.BRAKE)
        self.turningMotor.stopMotor()

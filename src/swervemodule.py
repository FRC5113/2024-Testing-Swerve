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
from wpilib import Preferences

from util import WPI_TalonFX


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
            0,
            0,
            0,
            wpimath.trajectory.TrapezoidProfile.Constraints(0, 0),
        )

        # Gains are for example purposes only - must be determined for your own robot!
        self.driveFeedforward = wpimath.controller.SimpleMotorFeedforwardMeters(0, 0)
        self.turnFeedforward = wpimath.controller.SimpleMotorFeedforwardMeters(0, 0)
        self.speedScale = 0
        self.initPreferences()

        # Limit the PID Controller's input range between -pi and pi and set the input
        # to be continuous.
        self.turningPIDController.enableContinuousInput(-math.pi, math.pi)

    def initPreferences(self) -> None:
        Preferences.initDouble("drive_kP", 0.0)
        Preferences.initDouble("drive_kS", 0.15)
        Preferences.initDouble("drive_kV", 0.102)
        Preferences.initDouble("turning_kP", 18.0)
        Preferences.initDouble("turning_kS", 0.14)
        Preferences.initDouble("turning_kV", 0.375)
        Preferences.initDouble("speed_scale", 1.0)
        Preferences.initDouble("turning_max_v", 30.0)
        Preferences.initDouble("turning_max_a", 300.0)

    def collectPreferences(self) -> None:
        # consider writing a "smart preference" class (ie with property) to make this less arduous
        self.drivePIDController.setP(Preferences.getDouble("drive_kP"))
        self.driveFeedforward = wpimath.controller.SimpleMotorFeedforwardMeters(
            Preferences.getDouble("drive_kS"), Preferences.getDouble("drive_kV")
        )
        self.turningPIDController.setP(Preferences.getDouble("turning_kP"))
        self.turnFeedforward = wpimath.controller.SimpleMotorFeedforwardMeters(
            Preferences.getDouble("turning_kS"), Preferences.getDouble("turning_kV")
        )
        self.speedScale = Preferences.getDouble("speed_scale")
        self.turningPIDController.setConstraints(
            wpimath.trajectory.TrapezoidProfile.Constraints(
                Preferences.getDouble("max_v"), Preferences.getDouble("max_a")
            )
        )

    def setDesiredState(
        self, desiredState: wpimath.kinematics.SwerveModuleState
    ) -> None:
        """Sets the desired state for the module.

        :param desiredState: Desired state with speed and angle.
        """

        # update controllers with new values from networktables
        self.collectPreferences()

        encoderRotation = wpimath.geometry.Rotation2d(
            self.turningEncoder.get_position().value * 2 * math.pi
        )

        # Optimize the reference state to avoid spinning further than 90 degrees
        state = wpimath.kinematics.SwerveModuleState.optimize(
            desiredState, encoderRotation
        )

        # Scale speed by cosine of angle error. This scales down movement perpendicular to the desired
        # direction of travel that can occur when modules change directions. This results in smoother
        # driving.
        state.speed *= (state.angle - encoderRotation).cos() * self.speedScale + (
            1 - self.speedScale
        )

        # Calculate the drive output from the drive PID controller.
        driveFeedback = self.drivePIDController.calculate(
            self.driveMotor.get_velocity().value, state.speed
        )

        driveFeedforward = self.driveFeedforward.calculate(state.speed)

        # Calculate the turning motor output (in rad/s) from the turning PID controller.
        turnFeedback = self.turningPIDController.calculate(
            self.turningEncoder.get_position().value * 2 * math.pi,
            state.angle.radians(),
        )

        # convert the supplied velocity from the PID controller to a voltage with the feedforward
        turnOutput = self.turnFeedforward.calculate(turnFeedback)

        self.driveMotor.setVoltage(driveFeedback + driveFeedforward)
        self.turningMotor.setVoltage(-turnOutput)
        # puts pid stuff in terminal - switch to networktables eventually
        if self.turningEncoder.device_id == 33:
            print(
                f"y: {encoderRotation.radians()}, r: {state.angle.radians()}, e: {encoderRotation.radians() - state.angle.radians()}, u: {turnFeedback}, ff: {turnOutput}"
            )

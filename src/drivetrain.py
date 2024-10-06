#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import math
import navx
from wpilib import Preferences, SmartDashboard
from wpiutil import Sendable, SendableBuilder
import wpimath.geometry
import wpimath.kinematics
import swervemodule


class Drivetrain(Sendable):
    """
    Represents a swerve drive style drivetrain.
    """

    def __init__(self) -> None:
        Sendable.__init__(self)
        self.frontLeftLocation = wpimath.geometry.Translation2d(0.318, 0.318)
        self.frontRightLocation = wpimath.geometry.Translation2d(0.318, -0.318)
        self.backLeftLocation = wpimath.geometry.Translation2d(-0.318, 0.318)
        self.backRightLocation = wpimath.geometry.Translation2d(-0.318, -0.318)
        self.driveGearRatio = 6.75
        self.wheelRadius = 0.0508

        self.frontLeft = swervemodule.SwerveModule(11, 12, 13)
        self.frontRight = swervemodule.SwerveModule(21, 22, 23)
        self.backLeft = swervemodule.SwerveModule(31, 32, 33)
        self.backRight = swervemodule.SwerveModule(41, 42, 43)

        self.gyro = navx.AHRS.create_spi()

        self.kinematics = wpimath.kinematics.SwerveDrive4Kinematics(
            self.frontLeftLocation,
            self.frontRightLocation,
            self.backLeftLocation,
            self.backRightLocation,
        )

        self.gyro.reset()

        Preferences.initDouble("max_speed", 3.0)
        self.maxSpeed = 3.0

        self.swerveModuleStates = self.kinematics.toSwerveModuleStates(
            wpimath.kinematics.ChassisSpeeds(0, 0, 0)
        )
        SmartDashboard.putData("Swerve Drive", self)

    def initSendable(self, builder: SendableBuilder) -> None:
        # Sendable.initSendable(self, builder)
        builder.setSmartDashboardType("SwerveDrive")
        builder.addDoubleProperty(
            "Front Left Velocity",
            lambda: self.swerveModuleStates[0].speed / self.maxSpeed / 3,
            lambda _: None,
        )
        builder.addDoubleProperty(
            "Front Left Angle",
            lambda: self.swerveModuleStates[0].angle.radians(),
            lambda _: None,
        )
        builder.addDoubleProperty(
            "Front Right Velocity",
            lambda: self.swerveModuleStates[1].speed / self.maxSpeed / 3,
            lambda _: None,
        )
        builder.addDoubleProperty(
            "Front Right Angle",
            lambda: self.swerveModuleStates[1].angle.radians(),
            lambda _: None,
        )
        builder.addDoubleProperty(
            "Back Left Velocity",
            lambda: self.swerveModuleStates[2].speed / self.maxSpeed / 3,
            lambda _: None,
        )
        builder.addDoubleProperty(
            "Back Left Angle",
            lambda: self.swerveModuleStates[2].angle.radians(),
            lambda _: None,
        )
        builder.addDoubleProperty(
            "Back Right Velocity",
            lambda: self.swerveModuleStates[3].speed / self.maxSpeed / 3,
            lambda _: None,
        )
        builder.addDoubleProperty(
            "Back Right Angle",
            lambda: self.swerveModuleStates[3].angle.radians(),
            lambda _: None,
        )

    def convertSpeed(self, linearSpeed: float) -> float:
        # converts linear speed (m/s) to angular speed of the motor (rad/s)
        # figure out how to use units!
        return linearSpeed * self.driveGearRatio / (self.wheelRadius * 2 * math.pi)

    def drive(
        self,
        xSpeed: float,
        ySpeed: float,
        rot: float,
        fieldRelative: bool,
        periodSeconds: float,
    ) -> None:
        """
        Method to drive the robot using joystick info.
        :param xSpeed: Speed of the robot in the x direction (forward).
        :param ySpeed: Speed of the robot in the y direction (sideways).
        :param rot: Angular rate of the robot.
        :param fieldRelative: Whether the provided x and y speeds are relative to the field.
        :param periodSeconds: Time
        """
        self.swerveModuleStates = self.kinematics.toSwerveModuleStates(
            wpimath.kinematics.ChassisSpeeds.discretize(
                (
                    wpimath.kinematics.ChassisSpeeds.fromFieldRelativeSpeeds(
                        xSpeed, ySpeed, rot, self.gyro.getRotation2d()
                    )
                    if fieldRelative
                    else wpimath.kinematics.ChassisSpeeds(xSpeed, ySpeed, rot)
                ),
                periodSeconds,
            )
        )
        wpimath.kinematics.SwerveDrive4Kinematics.desaturateWheelSpeeds(
            self.swerveModuleStates,
            Preferences.getDouble("max_speed") * 6.75 / (0.0508 * 2 * math.pi),
        )
        self.frontLeft.setDesiredState(self.swerveModuleStates[0])
        self.frontRight.setDesiredState(self.swerveModuleStates[1])
        self.backLeft.setDesiredState(self.swerveModuleStates[2])
        self.backRight.setDesiredState(self.swerveModuleStates[3])

    def reset_gyro(self) -> None:
        self.gyro.reset()

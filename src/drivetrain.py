#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import math
import navx
import ntcore
from wpilib import Preferences, SmartDashboard
from wpiutil import Sendable, SendableBuilder
import wpimath.geometry
import wpimath.kinematics
import swervemodule
import phoenix6

kMaxSpeed = 3.0 * 6.75 / (0.0508 * 2 * math.pi) # 3 meters per second
kMaxAngularSpeed = math.pi  # 1/2 rotation per second


class Drivetrain:
    """
    Represents a swerve drive style drivetrain.
    """

    def __init__(self) -> None:
        self.frontLeftLocation = wpimath.geometry.Translation2d(0.318, 0.318)
        self.frontRightLocation = wpimath.geometry.Translation2d(0.318, -0.318)
        self.backLeftLocation = wpimath.geometry.Translation2d(-0.318, 0.318)
        self.backRightLocation = wpimath.geometry.Translation2d(-0.318, -0.318)

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

        self.maxSpeed = Preferences.initDouble("max_speed", 3.0)

    def initSendable(self):
        self.swerveModuleStates = self.kinematics.toSwerveModuleStates(
            wpimath.kinematics.ChassisSpeeds(0, 0, 0)
        )

        def getSpeedGetter(i):
            def getter():
                return self.swerveModuleStates[i].speed
            return getter
        
        def getAngleGetter(i):
            return self.swerveModuleStates[i].angle.radians
        
        def dummySetter(value):
            pass

        class SwerveSendable(Sendable):
            def initSendable(self, builder: SendableBuilder):
                builder.setSmartDashboardType("SwerveDrive")
                builder.addDoubleProperty("Front Left Velocity", getSpeedGetter(0), dummySetter)
                builder.addDoubleProperty("Front Left Angle", getAngleGetter(0), dummySetter)
                builder.addDoubleProperty("Front Right Velocity", getSpeedGetter(1), dummySetter)
                builder.addDoubleProperty("Front Right Angle", getAngleGetter(1), dummySetter)
                builder.addDoubleProperty("Back Left Velocity", getSpeedGetter(2), dummySetter)
                builder.addDoubleProperty("Back Left Angle", getAngleGetter(2), dummySetter)
                builder.addDoubleProperty("Back Right Velocity", getSpeedGetter(3), dummySetter)
                builder.addDoubleProperty("Back Right Angle", getAngleGetter(3), dummySetter)

        self.sendable = SwerveSendable()
        SmartDashboard.putData("Swerve Drive", self.sendable)

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
            self.swerveModuleStates, Preferences.getDouble("max_speed")  * 6.75 / (0.0508 * 2 * math.pi)
        )
        self.frontLeft.setDesiredState(self.swerveModuleStates[0])
        self.frontRight.setDesiredState(self.swerveModuleStates[1])
        self.backLeft.setDesiredState(self.swerveModuleStates[2])
        self.backRight.setDesiredState(self.swerveModuleStates[3])
        

    def reset_gyro(self) -> None:
        self.gyro.reset()

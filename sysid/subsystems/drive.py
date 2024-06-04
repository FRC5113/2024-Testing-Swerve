#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

from commands2 import Command, Subsystem
from commands2.sysid import SysIdRoutine
from wpilib.sysid import SysIdRoutineLog

from wpilib import Encoder, PWMSparkMax, RobotController
from wpilib.drive import DifferentialDrive

from wpimath.kinematics import SwerveModuleState
from wpimath.geometry import Rotation2d
from wpimath.units import volts

from constants import DriveConstants
from subsystems import swervemodule

from typing import Callable


class Drive(Subsystem):
    def __init__(self) -> None:
        self.frontLeft = swervemodule.SwerveModule(11, 12, 13)
        self.frontRight = swervemodule.SwerveModule(21, 22, 23)
        self.backLeft = swervemodule.SwerveModule(31, 32, 33)
        self.backRight = swervemodule.SwerveModule(41, 42, 43)

        # Tell SysId how to plumb the driving voltage to the motors.
        def drive(voltage: volts) -> None:
            desiredState = SwerveModuleState(voltage, Rotation2d(0))
            self.frontLeft.setDesiredState(desiredState)
            self.frontRight.setDesiredState(desiredState)
            self.backLeft.setDesiredState(desiredState)
            self.backRight.setDesiredState(desiredState)

        # Tell SysId to make generated commands require this subsystem, suffix test state in
        # WPILog with this subsystem's name ("drive")
        self.sys_id_routine = SysIdRoutine(
            SysIdRoutine.Config(),
            SysIdRoutine.Mechanism(drive, self.log, self),
        )

    # Tell SysId how to record a frame of data for each motor on the mechanism being
    # characterized.
    def log(self, sys_id_routine: SysIdRoutineLog) -> None:
        # Record a frame for the left motors.  Since these share an encoder, we consider
        # the entire group to be one motor.
        sys_id_routine.motor("drive-left").voltage(
            self.frontLeft.getDriveVoltage()
        ).position(self.frontLeft.getDrivePosition()).velocity(
            self.frontLeft.getDriveVelocity()
        )
        # Record a frame for the right motors.  Since these share an encoder, we consider
        # the entire group to be one motor.
        sys_id_routine.motor("drive-right").voltage(
            self.frontRight.getDriveVoltage()
        ).position(self.frontRight.getDrivePosition()).velocity(
            self.frontRight.getDriveVelocity()
        )

    # def arcadeDriveCommand(
    #     self, fwd: Callable[[], float], rot: Callable[[], float]
    # ) -> Command:
    #     """Returns a command that drives the robot with arcade controls.

    #     :param fwd: the commanded forward movement
    #     :param rot: the commanded rotation
    #     """

    #     # A split-stick arcade command, with forward/backward controlled by the left
    #     # hand, and turning controlled by the right.
    #     return self.run(lambda: self.drive.arcadeDrive(fwd(), rot()))

    def sysIdQuasistatic(self, direction: SysIdRoutine.Direction) -> Command:
        return self.sys_id_routine.quasistatic(direction)

    def sysIdDynamic(self, direction: SysIdRoutine.Direction) -> Command:
        return self.sys_id_routine.dynamic(direction)

import wpilib
from wpilib import Preferences
import wpimath
import wpilib.drive
import wpimath.filter
import wpimath.controller

import drivetrain


class MyRobot(wpilib.TimedRobot):
    def robotInit(self) -> None:
        """Robot initialization function"""
        self.controller = wpilib.PS5Controller(0)
        self.swerve = drivetrain.Drivetrain()

        # Slew rate limiters to make joystick inputs more gentle
        wpilib.Preferences.initDouble("slew_rate", 5)
        self.slewRate = wpilib.Preferences.getDouble("slew_rate")
        self.xspeedLimiter = wpimath.filter.SlewRateLimiter(self.slewRate)
        self.yspeedLimiter = wpimath.filter.SlewRateLimiter(self.slewRate)
        self.rotLimiter = wpimath.filter.SlewRateLimiter(self.slewRate)

    def autonomousPeriodic(self) -> None:
        self.driveWithJoystick(True)

    def teleopPeriodic(self) -> None:
        if self.slewRate != wpilib.Preferences.getDouble("slew_rate"):
            self.slewRate = wpilib.Preferences.getDouble("slew_rate")
            self.xspeedLimiter = wpimath.filter.SlewRateLimiter(self.slewRate)
            self.yspeedLimiter = wpimath.filter.SlewRateLimiter(self.slewRate)
            self.rotLimiter = wpimath.filter.SlewRateLimiter(self.slewRate)

        self.driveWithJoystick(True)
        if self.controller.getCreateButton():
            self.swerve.reset_gyro()

    def driveWithJoystick(self, fieldRelative: bool) -> None:
        # Get the x speed. We are inverting this because Xbox controllers return
        # negative values when we push forward.
        xSpeed = -self.xspeedLimiter.calculate(
            wpimath.applyDeadband(self.controller.getLeftY(), 0.1)
        ) * self.swerve.convertSpeed(Preferences.getDouble("max_speed"))

        # Get the y speed or sideways/strafe speed. We are inverting this because
        # we want a positive value when we pull to the left. Xbox controllers
        # return positive values when you pull to the right by default.
        ySpeed = -self.yspeedLimiter.calculate(
            wpimath.applyDeadband(self.controller.getLeftX(), 0.1)
        ) * self.swerve.convertSpeed(Preferences.getDouble("max_speed"))

        # Get the rate of angular rotation. We are inverting this because we want a
        # positive value when we pull to the left (remember, CCW is positive in
        # mathematics). Xbox controllers return positive values when you pull to
        # the right by default.
        rot = -self.rotLimiter.calculate(
            wpimath.applyDeadband(self.controller.getRightX(), 0.1)
        ) * self.swerve.convertSpeed(Preferences.getDouble("max_speed"))

        self.swerve.drive(xSpeed, ySpeed, rot, fieldRelative, self.getPeriod())

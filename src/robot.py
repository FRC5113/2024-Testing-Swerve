import math

import wpilib
import wpimath
import wpilib.drive
import wpimath.filter
import wpimath.controller
import phoenix6

import drivetrain
import swervemodule
import util


class MyRobot(wpilib.TimedRobot):
    def robotInit(self) -> None:
        """Robot initialization function"""
        self.controller = wpilib.XboxController(0)
        self.swerve = drivetrain.Drivetrain()

        # Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
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
        if self.controller.getStartButton():
            self.swerve.reset_gyro()

    def driveWithJoystick(self, fieldRelative: bool) -> None:
        # Get the x speed. We are inverting this because Xbox controllers return
        # negative values when we push forward.
        xSpeed = (
            -self.xspeedLimiter.calculate(
                wpimath.applyDeadband(self.controller.getLeftY(), 0.02)
            )
            * drivetrain.kMaxSpeed
        )

        # Get the y speed or sideways/strafe speed. We are inverting this because
        # we want a positive value when we pull to the left. Xbox controllers
        # return positive values when you pull to the right by default.
        ySpeed = (
            -self.yspeedLimiter.calculate(
                wpimath.applyDeadband(self.controller.getLeftX(), 0.02)
            )
            * drivetrain.kMaxSpeed
        )

        # Get the rate of angular rotation. We are inverting this because we want a
        # positive value when we pull to the left (remember, CCW is positive in
        # mathematics). Xbox controllers return positive values when you pull to
        # the right by default.
        rot = (
            -self.rotLimiter.calculate(
                wpimath.applyDeadband(self.controller.getRightX(), 0.02)
            )
            * drivetrain.kMaxSpeed
        )

        if self.controller.getAButton():
            self.swerve.drive(1, 0, 0, False, self.getPeriod())
            return

        if self.controller.getBButton():
            self.swerve.drive(0, 1, 0, False, self.getPeriod())
            return

        self.swerve.drive(xSpeed, ySpeed, rot, fieldRelative, self.getPeriod())

    def testInit(self) -> None:
        self.test_motor = swervemodule.WPI_TalonFX(12)
        self.test_encoder = phoenix6.hardware.CANcoder(13)
        self.test_voltage = 0
        self.test_kS = 0
        self.test_kV = 0

    def testPeriodic(self) -> None:
        # routine to determine the kS and kV of a motor

        # minimum velocity that is considered "moving" (rad/s)
        v_threshold = 0.01  # not tuned
        # amount by which voltage increases each cycle
        voltage_step = 0.01

        self.test_motor.setVoltage(self.test_voltage)
        velocity = self.test_encoder.get_velocity().value * 2 * math.pi
        if velocity > v_threshold and self.test_kS == 0:
            self.test_kS = self.test_voltage

        if self.test_kS > 0:
            self.test_kV = (self.test_voltage - self.test_kS) / velocity

        print(f"measured kS: {self.test_kS}, measured kV: {self.test_kV}")

        self.test_voltage += voltage_step

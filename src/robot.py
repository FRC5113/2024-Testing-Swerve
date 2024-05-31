import math

import wpilib
import wpimath
import wpilib.drive
import wpimath.filter
import wpimath.controller
import phoenix6

import drivetrain
import swervemodule


class MyRobot(wpilib.TimedRobot):
    def robotInit(self) -> None:
        """Robot initialization function"""
        self.controller = wpilib.XboxController(0)
        self.swerve = drivetrain.Drivetrain()

        # Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
        self.xspeedLimiter = wpimath.filter.SlewRateLimiter(3)
        self.yspeedLimiter = wpimath.filter.SlewRateLimiter(3)
        self.rotLimiter = wpimath.filter.SlewRateLimiter(3)

    def autonomousPeriodic(self) -> None:
        self.driveWithJoystick(True)

    def teleopPeriodic(self) -> None:
        self.driveWithJoystick(True)
        if self.controller.getStartButtonPressed():
            self.swerve.reset_gyro()

    def driveWithJoystick(self, fieldRelative: bool) -> None:
        # Get the x speed. We are inverting this because Xbox controllers return
        # negative values when we push forward.
        xSpeed = (
            -self.xspeedLimiter.calculate(self.controller.getLeftY())
            * drivetrain.kMaxSpeed
        )

        # Get the y speed or sideways/strafe speed. We are inverting this because
        # we want a positive value when we pull to the left. Xbox controllers
        # return positive values when you pull to the right by default.
        ySpeed = (
            -self.yspeedLimiter.calculate(self.controller.getLeftX())
            * drivetrain.kMaxSpeed
        )

        # deadband of radius 0.1
        if xSpeed ** 2 + ySpeed ** 2 < 0.1 **2:
            xSpeed = 0
            ySpeed = 0

        # Get the rate of angular rotation. We are inverting this because we want a
        # positive value when we pull to the left (remember, CCW is positive in
        # mathematics). Xbox controllers return positive values when you pull to
        # the right by default.
        rot = (
            -self.rotLimiter.calculate(
                wpimath.applyDeadband(self.controller.getRightX(), 0.1)
            )
            * drivetrain.kMaxSpeed
        )

        if self.controller.getAButton():
            self.swerve.drive(-3, 0, 0, False, self.getPeriod())
            return

        if self.controller.getBButton():
            self.swerve.drive(0, -3, 0, False, self.getPeriod())
            return
        
        if self.controller.getXButton():
            self.swerve.drive(0, 3, 0, False, self.getPeriod())
            return

        if self.controller.getYButton():
            self.swerve.drive(3, 0, 0, False, self.getPeriod())
            return

        self.swerve.drive(xSpeed, ySpeed, rot, fieldRelative, self.getPeriod())

    def testInit(self) -> None:
        self.test_motor = swervemodule.WPI_TalonFX(12)
        self.test_encoder = phoenix6.hardware.CANcoder(13)
        self.test_voltage = 0
        self.test_kS = 0
        self.test_kV = 0
        self.test_voltage_step = 0.001

    def testPeriodic(self) -> None:
        # routine to determine the kS and kV of a motor

        # minimum velocity that is considered "moving" (rad/s)
        v_threshold = 0.05  # not tuned
        # amount by which voltage increases each cycle

        self.test_motor.setVoltage(self.test_voltage)
        velocity = self. test_motor.get_velocity().value * 2 * math.pi#-self.test_encoder.get_velocity().value * 2 * math.pi
        if velocity > v_threshold and self.test_kS == 0:
            self.test_kS = self.test_voltage
            self.test_voltage_step = 0.01

        if self.test_kS > 0 and velocity != 0:
            self.test_kV = (self.test_voltage - self.test_kS) / velocity

        print(f"measured kS: {self.test_kS}, measured kV: {self.test_kV}, measured velocity: {velocity}")

        self.test_voltage += self.test_voltage_step

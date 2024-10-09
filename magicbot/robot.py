import math

import wpilib.shuffleboard

from components.swerve_drive import SwerveDrive
from components.swerve_wheel import SwerveWheel
from phoenix6.hardware import TalonFX
from phoenix6.hardware import CANcoder
import magicbot
import navx
import wpilib
from wpimath import applyDeadband
from wpilib import SmartDashboard, RobotController, SendableChooser
from wpilib.shuffleboard import BuiltInWidgets, Shuffleboard

from utility import SmartPreference, SmartProfile


class MyRobot(magicbot.MagicRobot):
    swerve_drive: SwerveDrive
    front_left: SwerveWheel
    front_right: SwerveWheel
    rear_left: SwerveWheel
    rear_right: SwerveWheel

    """This should be the max speed (m/s) at which the drive motors can
    run, NOT the max speed that the robot should go (ie. use a curve
    instead). This is because this is also used to calculate omega."""
    max_speed = SmartPreference(3.0)

    def createObjects(self):
        self.debug = True

        # Swerve Motor IDs
        self.front_left_speed_motor = TalonFX(11)
        self.front_left_direction_motor = TalonFX(12)
        self.front_left_cancoder = CANcoder(13)

        self.front_right_speed_motor = TalonFX(21)
        self.front_right_direction_motor = TalonFX(22)
        self.front_right_cancoder = CANcoder(23)

        self.rear_left_speed_motor = TalonFX(31)
        self.rear_left_direction_motor = TalonFX(32)
        self.rear_left_cancoder = CANcoder(33)

        self.rear_right_speed_motor = TalonFX(41)
        self.rear_right_direction_motor = TalonFX(42)
        self.rear_right_cancoder = CANcoder(43)

        # Swerve Drive
        self.navX = navx.AHRS.create_spi()
        self.offset_x = 0.381
        self.offset_y = 0.381
        self.drive_gear_ratio = 6.75
        self.wheel_radius = 0.0508

        self.speed_profile = SmartProfile("speed")
        self.direction_profile = SmartProfile(
            "direction", continuous_range=(0, math.tau)
        )
        SmartDashboard.putData("Speed Profile", self.speed_profile)
        SmartDashboard.putData("Direction Profile", self.direction_profile)

        # controller chooser
        self.controller = SendableChooser()
        self.controller.setDefaultOption("Playstation", wpilib.PS5Controller)
        self.controller.addOption("Xbox", wpilib.XboxController)

        SmartDashboard.putData("Controller", self.controller)

    def _simulationInit(self):
        self.field = wpilib.Field2d()

    def teleopPeriodic(self):
        if self.controller.getSelected() == wpilib.XboxController:
            self.driver_controller = wpilib.XboxController(0)
        elif self.controller.getSelected() == wpilib.PS5Controller:
            self.driver_controller = wpilib.PS5Controller(0)

            # Initialize Xbox and PS5 controllers as instance variables
        if isinstance(self.driver_controller, wpilib.XboxController):
            self.leftbumper = self.driver_controller.getLeftBumper()
            self.rightbumper = self.driver_controller.getRightBumper()
            self.startbutton = self.driver_controller.getStartButton()

        elif isinstance(self.driver_controller, wpilib.PS5Controller):
            self.leftbumper = self.driver_controller.getL1Button()
            self.rightbumper = self.driver_controller.getR1Button()
            self.startbutton = self.driver_controller.getOptionsButton()
        print(self.driver_controller)
        mult = 1
        if self.leftbumper:
            mult *= 0.5
        if self.rightbumper:
            mult *= 0.5

        """x is forward/backward, y is left/right. invert both axes for
        correct orientation"""
        left_joy_x = (
            applyDeadband(-self.driver_controller.getLeftY(), 0.1)
            * mult
            * self.max_speed
        )
        left_joy_y = (
            applyDeadband(-self.driver_controller.getLeftX(), 0.1)
            * mult
            * self.max_speed
        )
        # Define the POV-to-(left_joy_x, left_joy_y) mapping
        pov_mapping = {
            0: (1, 0),
            45: (0.707, -0.707),
            90: (0, -1),
            135: (-0.707, -0.707),
            180: (-1, 0),
            225: (-0.707, 0.707),
            270: (0, 1),
            315: (0.707, 0.707),
        }

        # Get the current POV from the controller
        pov_value = self.driver_controller.getPOV()

        # Update the joystick values based on the POV value if it's in the mapping
        if pov_value in pov_mapping:
            left_joy_x, left_joy_y = pov_mapping[pov_value]
            left_joy_x *= mult * self.max_speed
            left_joy_y *= mult * self.max_speed

        # calculate max angular speed based on max_speed (cool math here)
        omega = self.max_speed / math.dist((0, 0), (self.offset_x, self.offset_y))
        right_joy_x = (
            -applyDeadband(self.driver_controller.getRightX(), 0.1) * mult * omega
        )

        if left_joy_x != 0 or left_joy_y != 0 or right_joy_x != 0:
            self.swerve_drive.drive(
                left_joy_x, left_joy_y, right_joy_x, self.max_speed, self.period
            )

        if self.startbutton:
            self.swerve_drive.reset_gyro()

        SmartDashboard.putNumber("Gyro Angle", self.navX.getAngle())
        SmartDashboard.putNumber("Voltage", RobotController.getBatteryVoltage())

    def _simulationPeriodic(self):
        self.swerve_drive.update_odometry()
        self.field.setRobotPose(self.swerve_drive.odometry.getPose())
        SmartDashboard.putData("Field", self.field)

    # override _do_periodics() to access watchdog
    # DON'T DO ANYTHING ELSE HERE UNLESS YOU KNOW WHAT YOU'RE DOING
    def _do_periodics(self):
        super()._do_periodics()
        self.period = max(0.02, self.watchdog.getTime())


if __name__ == "__main__":
    wpilib.run(MyRobot)

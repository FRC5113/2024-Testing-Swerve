import math

import wpilib.shuffleboard

import wpilib.shuffleboard

from phoenix6.hardware import TalonFX
from phoenix6.hardware import CANcoder
import magicbot
import navx
import wpilib
from wpimath import applyDeadband
from wpilib import SmartDashboard, RobotController, SendableChooser
from robotpy_apriltag import AprilTagField, loadAprilTagLayoutField

from components.sysid_drive import SysIdDrive
from components.swerve_drive import SwerveDrive
from components.swerve_wheel import SwerveWheel
from components.vision import Vision
from util.smart_preference import SmartPreference, SmartProfile
from util.wrappers import LemonCamera


class MyRobot(magicbot.MagicRobot):
    sysid_drive: SysIdDrive

    swerve_drive: SwerveDrive
    front_left: SwerveWheel
    front_right: SwerveWheel
    rear_left: SwerveWheel
    rear_right: SwerveWheel
    vision: Vision

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

        # vision
        self.camera = LemonCamera(

        )
        self.field_layout = loadAprilTagLayoutField(AprilTagField.k2024Crescendo)

        # controller chooser
        self.controller = SendableChooser()
        self.controller.setDefaultOption("Playstation", wpilib.PS5Controller)
        self.controller.addOption("Xbox", wpilib.XboxController)

        SmartDashboard.putData("Controller", self.controller)

        self.estimated_field = wpilib.Field2d()

    def teleopPeriodic(self):
        # update camera
        if self.isReal():
            self.camera.update()

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

        # Get the current POV from the controller
        pov_value = self.driver_controller.getPOV()
        if pov_value >= 0:
            left_joy_x = math.cos(pov_value * math.pi / 180) * mult * self.max_speed
            left_joy_x = -math.sin(pov_value * math.pi / 180) * mult * self.max_speed

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

        if isinstance(self.driver_controller, wpilib.XboxController):
            if self.driver_controller.getAButton():
                self.sysid_drive.quasistatic_forward()
            if self.driver_controller.getBButton():
                self.sysid_drive.quasistatic_reverse()
            if self.driver_controller.getXButton():
                self.sysid_drive.dynamic_forward()
            if self.driver_controller.getYButton():
                self.sysid_drive.dynamic_reverse()

        SmartDashboard.putNumber("Gyro Angle", self.navX.getAngle())
        SmartDashboard.putNumber("Voltage", RobotController.getBatteryVoltage())
        self.estimated_field.setRobotPose(self.swerve_drive.get_estimated_pose())
        SmartDashboard.putData("Estimated Field", self.estimated_field)

    # override _do_periodics() to access watchdog
    # DON'T DO ANYTHING ELSE HERE UNLESS YOU KNOW WHAT YOU'RE DOING
    def _do_periodics(self):
        super()._do_periodics()
        self.period = max(0.02, self.watchdog.getTime())


if __name__ == "__main__":
    wpilib.run(MyRobot)

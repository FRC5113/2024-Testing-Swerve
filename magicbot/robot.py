import math

from components.swerve_drive import SwerveDrive
from components.swerve_wheel import SwerveWheel
from phoenix6.hardware import TalonFX
from phoenix6.hardware import CANcoder
from phoenix6.configs.talon_fx_configs import TalonFXConfiguration
import magicbot
import navx
import wpilib
from wpimath import applyDeadband
from wpilib import SmartDashboard, RobotController

from utility import SmartPreference


class MyRobot(magicbot.MagicRobot):
    swerve_drive: SwerveDrive
    front_left: SwerveWheel
    front_right: SwerveWheel
    rear_left: SwerveWheel
    rear_right: SwerveWheel

    # initialize SmartPreferences as class attributes
    direction_kS = SmartPreference(0.14)
    direction_kP = SmartPreference(18.0)
    direction_kI = SmartPreference(0)
    direction_kD = SmartPreference(0)
    direction_kV = SmartPreference(0.375)
    direction_kA = SmartPreference(0)
    direction_kMaxV = SmartPreference(0)
    speed_kS = SmartPreference(0.15)
    speed_kV = SmartPreference(0.102)
    speed_kA = SmartPreference(0)
    speed_kP = SmartPreference(0)
    speed_kI = SmartPreference(0)
    speed_kD = SmartPreference(0)
    speed_kMaxA = SmartPreference(400)
    speed_kMaxJ = SmartPreference(4000)
    """This should be the max speed (m/s) at which the drive motors can
    run, NOT the max speed that the robot should go (ie. use a curve
    instead). This is because this is also used to calculate omega."""
    max_speed = SmartPreference(3.0)

    def createObjects(self):
        self.debug = True

        # Swerve Motor IDs
        self.front_left_speed_motor = TalonFX(12)
        self.front_left_direction_motor = TalonFX(11)
        self.front_left_cancoder = CANcoder(13)

        self.front_right_speed_motor = TalonFX(22)
        self.front_right_direction_motor = TalonFX(21)
        self.front_right_cancoder = CANcoder(23)

        self.rear_left_speed_motor = TalonFX(32)
        self.rear_left_direction_motor = TalonFX(31)
        self.rear_left_cancoder = CANcoder(33)

        self.rear_right_speed_motor = TalonFX(42)
        self.rear_right_direction_motor = TalonFX(41)
        self.rear_right_cancoder = CANcoder(43)

        # Swerve Motor Configs
        self.fetch_swerve_motor_configs()

        # Swerve Drive
        self.navX = navx.AHRS.create_spi()
        self.offset_x = 0.381
        self.offset_y = 0.381
        self.drive_gear_ratio = 6.75
        self.wheel_radius = 0.0508

        # Controller
        self.driver_controller = wpilib.XboxController(0)

    def fetch_swerve_motor_configs(self):
        """Fetch SmartPreferences for swerve motor configs"""
        self.direction_configs = TalonFXConfiguration()
        self.direction_configs.slot0.k_s = self.direction_kS
        self.direction_configs.slot0.k_p = self.direction_kP
        self.direction_configs.slot0.k_i = self.direction_kI
        self.direction_configs.slot0.k_d = self.direction_kD
        self.direction_configs.motion_magic.motion_magic_expo_k_v = self.direction_kV
        self.direction_configs.motion_magic.motion_magic_expo_k_a = self.direction_kA
        self.direction_configs.motion_magic.motion_magic_cruise_velocity = (
            self.direction_kMaxV
        )

        self.speed_configs = TalonFXConfiguration()
        self.speed_configs.slot0.k_s = self.speed_kS
        self.speed_configs.slot0.k_v = self.speed_kS
        self.speed_configs.slot0.k_a = self.speed_kS
        self.speed_configs.slot0.k_p = self.speed_kP
        self.speed_configs.slot0.k_i = self.speed_kI
        self.speed_configs.slot0.k_d = self.speed_kD
        self.speed_configs.motion_magic.motion_magic_acceleration = self.speed_kMaxA
        self.speed_configs.motion_magic.motion_magic_jerk = self.speed_kMaxJ

    def _simulationInit(self):
        self.field = wpilib.Field2d()

    def teleopPeriodic(self):
        # called periodically so that NT updates can be read
        self.fetch_swerve_motor_configs()
        if SmartPreference.has_changed():
            self.front_left.hasUpdate()
            self.front_right.hasUpdate()
            self.rear_left.hasUpdate()
            self.rear_right.hasUpdate()

        mult = 1
        if self.driver_controller.getLeftBumper():
            mult *= 0.5
        if self.driver_controller.getRightBumper():
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
        # calculate max angular speed based on max_speed (cool math here)
        omega = self.max_speed / math.dist((0, 0), (self.offset_x, self.offset_y))
        right_joy_x = (
            applyDeadband(self.driver_controller.getRightX(), 0.1) * mult * omega
        )

        if left_joy_x != 0 or left_joy_y != 0 or right_joy_x != 0:
            self.swerve_drive.drive(
                left_joy_x, left_joy_y, right_joy_x, self.max_speed, self.period
            )

        if self.driver_controller.getStartButton():
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

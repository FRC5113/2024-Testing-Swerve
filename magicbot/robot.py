from components.swerve_drive import SwerveDrive
from components.swerve_wheel import SwerveWheel
from phoenix6.hardware import TalonFX
from phoenix6.hardware import CANcoder
from phoenix6.configs.talon_fx_configs import TalonFXConfiguration
import magicbot
import navx
import wpilib
from wpimath import applyDeadband
from wpilib import Preferences, SmartDashboard, RobotController, PowerDistribution


class MyRobot(magicbot.MagicRobot):
    swerve_drive: SwerveDrive
    front_left: SwerveWheel
    front_right: SwerveWheel
    rear_left: SwerveWheel
    rear_right: SwerveWheel

    def createObjects(self):
        self.debug = True

        # Swerve Wheels
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

        # Motor Configurations (replace with preferences eventually)
        self.direction_configs = TalonFXConfiguration()
        self.direction_configs.slot0.k_s = Preferences.getDouble("direction_kS")
        self.direction_configs.slot0.k_p = Preferences.getDouble("direction_kP")
        self.direction_configs.slot0.k_v = Preferences.getDouble("direction_kV")
        self.direction_configs.motion_magic.motion_magic_cruise_velocity = 0
        self.direction_configs.motion_magic.motion_magic_expo_k_v = (
            Preferences.getDouble("direction_Expo_kV")
        )
        self.direction_configs.motion_magic.motion_magic_expo_k_a = (
            Preferences.getDouble("direction_Expo_kA")
        )

        self.speed_configs = TalonFXConfiguration()
        self.speed_configs.slot0.k_s = Preferences.getDouble("speed_kS")
        self.speed_configs.slot0.k_v = Preferences.getDouble("speed_kV")
        self.speed_configs.slot0.k_p = Preferences.getDouble("speed_kP")
        self.speed_configs.motion_magic.motion_magic_acceleration = (
            Preferences.getDouble("speed_acceleration")
        )
        self.speed_configs.motion_magic.motion_magic_jerk = Preferences.getDouble(
            "speed_jerk"
        )

        # Swerve Drive
        self.navX = navx.AHRS.create_spi()
        self.offset_x = 0.381
        self.offset_y = 0.381
        self.drive_gear_ratio = 6.75
        self.wheel_radius = 0.0508
        self.max_speed = Preferences.getDouble("speed_scale")

        # Controller
        self.driver_controller = wpilib.XboxController(0)

        self.initPreferences()

    def teleopPeriodic(self):
        multiplier = self.max_speed  # todo: final touches, max speed in co, other prefs
        if self.driver_controller.getLeftBumper():
            multiplier *= 0.5
        if self.driver_controller.getRightBumper():
            multiplier *= 0.5

        left_joy_x = applyDeadband(self.driver_controller.getLeftX(), 0.1) * multiplier
        left_joy_y = applyDeadband(self.driver_controller.getLeftY(), 0.1) * multiplier
        right_joy_x = (
            applyDeadband(self.driver_controller.getRightX(), 0.1) * multiplier
        )

        if left_joy_x == 0 and left_joy_y == 0 and right_joy_x == 0:
            self.swerve_drive.freeze()
        else:
            self.swerve_drive.unfreeze()

        # assumes dt=0.02 (magicbot moment)
        self.swerve_drive.drive(left_joy_x, left_joy_y, right_joy_x, multiplier, 0.02)

        if self.driver_controller.getStartButton():
            self.swerve_drive.reset_gyro()

        SmartDashboard.putNumber("Gyro Angle", self.navX.getAngle())
        SmartDashboard.putNumber("Voltage", RobotController.getBatteryVoltage())

    def initPreferences(self) -> None:
        Preferences.initDouble("speed_kP", 0.0)
        Preferences.initDouble("speed_kS", 0.15)
        Preferences.initDouble("speed_kV", 0.102)
        Preferences.initDouble("speed_jerk", 4000)
        Preferences.initDouble("speed_acceleration", 400)
        Preferences.initDouble("speed_scale", 1.0)

        Preferences.initDouble("direction_kP", 18.0)
        Preferences.initDouble("direction_kS", 0.14)
        Preferences.initDouble("direction_kV", 0.375)
        Preferences.initDouble("direction_Expo_kA", 0.0)
        Preferences.initDouble("direction_Expo_kV", 0.12)


if __name__ == "__main__":
    wpilib.run(MyRobot)

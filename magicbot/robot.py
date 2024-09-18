from components.swerve_drive import SwerveDrive
from components.swerve_wheel import SwerveWheel
from phoenix6.hardware import TalonFX
from phoenix6.hardware import CANcoder
from phoenix6.configs.talon_fx_configs import TalonFXConfiguration
import magicbot
import navx
import wpilib
from wpimath import applyDeadband


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
        self.direction_configs.slot0.k_s = 0.25
        self.direction_configs.slot0.k_p = 4.8
        self.direction_configs.slot0.k_i = 0
        self.direction_configs.slot0.k_d = 0
        self.direction_configs.motion_magic.motion_magic_cruise_velocity = 0
        self.direction_configs.motion_magic.motion_magic_expo_k_v = 0.12
        self.direction_configs.motion_magic.motion_magic_expo_k_a = 0

        self.speed_configs = TalonFXConfiguration()
        self.speed_configs.slot0.k_s = 0.25
        self.speed_configs.slot0.k_v = 0.12
        self.speed_configs.slot0.k_a = 0
        self.speed_configs.slot0.k_p = 4.8
        self.speed_configs.slot0.k_i = 0
        self.speed_configs.slot0.k_d = 0
        self.speed_configs.motion_magic.motion_magic_acceleration = 400
        self.speed_configs.motion_magic.motion_magic_jerk = 4000

        # Swerve Drive
        self.navX = navx.AHRS.create_spi()
        self.offset_x = 0.381
        self.offset_y = 0.381
        self.drive_gear_ratio = 6.75
        self.wheel_radius = 0.0508
        self.max_speed = 3.0

        # Controller
        self.driver_controller = wpilib.XboxController(0)

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


if __name__ == "__main__":
    wpilib.run(MyRobot)

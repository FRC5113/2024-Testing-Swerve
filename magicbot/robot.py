import math
from pathlib import Path

from phoenix6.hardware import TalonFX
from phoenix6.hardware import CANcoder
import magicbot
from magicbot import feedback
import navx
import wpilib
from wpimath import applyDeadband, units
from wpimath.geometry import Transform3d
from wpilib import RobotController
from robotpy_apriltag import AprilTagField, loadAprilTagLayoutField, AprilTagFieldLayout

from components.odometry import Odometry
from components.swerve_drive import SwerveDrive
from components.swerve_wheel import SwerveWheel
from util.alerts import AlertType, AlertManager
from util.smart_preference import SmartPreference, SmartProfile
from util.camera import LemonCamera, LemonCameraSim
from util.input import LemonInput

# from container import RobotContainer


class MyRobot(magicbot.MagicRobot):
    odometry: Odometry

    swerve_drive: SwerveDrive
    front_left: SwerveWheel
    front_right: SwerveWheel
    rear_left: SwerveWheel
    rear_right: SwerveWheel

    low_bandwidth = False
    # greatest speed that chassis should move (not greatest possible speed)
    top_speed = SmartPreference(3.0)
    top_omega = SmartPreference(6.0)

    def createObjects(self):
        """This method is where all attributes to be injected are
        initialized. This is done here rather that inside the components
        themselves so that all constants and initialization parameters
        can be found in one place. Also, attributes shared by multiple
        components, such as the NavX, need only be created once.
        """

        # swerve motors and cancoders
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

        # NavX IMU
        self.navX = navx.AHRS.create_spi()

        # swerve constants
        self.offset_x = 0.381
        self.offset_y = 0.381
        self.drive_gear_ratio = 6.75
        self.wheel_radius = 0.0508
        self.max_speed = 4.7

        # swerve module profiles
        self.speed_profile = SmartProfile("speed", low_bandwidth=self.low_bandwidth)
        self.direction_profile = SmartProfile(
            "direction",
            continuous_range=(0, math.tau),
            low_bandwidth=self.low_bandwidth,
        )

        # odometry
        self.field_layout = AprilTagFieldLayout(
            str(Path(__file__).parent.resolve()) + "\\test_field.json"
        )
        if self.isSimulation():
            self.camera = LemonCameraSim(
                # loadAprilTagLayoutField(AprilTagField.k2024Crescendo), 120
                self.field_layout, 120
            )
        else:
            self.camera = LemonCamera("USB_Camera", Transform3d())
        self.theta_profile = SmartProfile(
            "theta",
            kP=0.05,
            continuous_range=(-180, 180),
            low_bandwidth=self.low_bandwidth,
        )

        # initialize AlertManager with logger (kinda bad code)
        AlertManager(self.logger)
        if self.low_bandwidth:
            AlertManager.instant_alert(
                "Low Bandwidth Mode is active! Tuning is disabled.", AlertType.WARNING
            )

    def teleopPeriodic(self):
        controller = LemonInput(0)

        mult = 1
        if controller.lefttrigger() >= 0.8:
            mult *= 0.5
        if controller.righttrigger() >= 0.8:
            mult *= 0.5

        if controller.pov() >= 0:
            # use pov inputs to steer if present
            self.swerve_drive.drive(
                controller.pov_y() * mult * self.top_speed,
                -controller.pov_x() * mult * self.top_speed,
                -applyDeadband(controller.rightx(), 0.1) * mult * self.top_omega,
                not controller.leftbumper(),
                self.period,
            )
        else:
            # otherwise steer with joysticks
            self.swerve_drive.drive(
                -applyDeadband(controller.lefty(), 0.1) * mult * self.top_speed,
                applyDeadband(controller.leftx(), 0.1) * mult * self.top_speed,
                -applyDeadband(controller.rightx(), 0.1) * mult * self.top_omega,
                not controller.leftbumper(),
                self.period,
            )

        if controller.abutton():
            self.odometry.face_tag()

        if controller.startbutton():
            self.swerve_drive.reset_gyro()

    @feedback
    def get_voltage(self) -> units.volts:
        return RobotController.getBatteryVoltage()

    # override _do_periodics() to access watchdog
    # DON'T DO ANYTHING ELSE HERE UNLESS YOU KNOW WHAT YOU'RE DOING
    def _do_periodics(self):
        super()._do_periodics()
        self.period = max(0.02, self.watchdog.getTime())


if __name__ == "__main__":
    wpilib.run(MyRobot)

import math

from components.swerve_drive import SwerveDrive
from components.swerve_wheel import SwerveWheel
from phoenix6.hardware import TalonFX
from phoenix6.hardware import CANcoder
import magicbot
import navx
import wpilib
from wpimath import applyDeadband
from wpimath.geometry import Translation2d
from wpilib import SmartDashboard, RobotController
from robotpy_apriltag import AprilTagField, loadAprilTagLayoutField

from components.vision import Vision
from components.swerve_drive import SwerveDrive
from components.swerve_wheel import SwerveWheel
from util.alerts import Alert, AlertType, AlertManager
from util.smart_preference import SmartPreference, SmartProfile
from util.wrappers import LemonCamera, LemonCameraSim, LemonInput

# from container import RobotContainer


class MyRobot(magicbot.MagicRobot):
    vision: Vision

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

        # self.contanier = RobotContainer()
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

        # vision and odometry
        self.estimated_field = wpilib.Field2d()
        self.tag_object = self.estimated_field.getObject("tag")
        if self.isSimulation():
            self.camera = LemonCameraSim(
                loadAprilTagLayoutField(AprilTagField.k2024Crescendo), 120
            )
        else:
            self.camera = LemonCamera("USB_Camera", Translation2d(0, 0), 0)
        self.field_layout = loadAprilTagLayoutField(AprilTagField.k2024Crescendo)

        # alerts
        SmartDashboard.putData("Alerts", AlertManager(self.logger))
        self.navx_alert = Alert(
            "NavX heading has been reset", AlertType.INFO, timeout=3.0
        )

    def teleopInit(self):
        self.navX.reset()
        self.navX.setAngleAdjustment(-90)

    def teleopPeriodic(self):
        controller = LemonInput(0)

        mult = 1
        # Call bumper methods on the instance
        if controller.lefttrigger() >= 0.8:
            mult *= 0.5
        if controller.righttrigger() >= 0.8:
            mult *= 0.5

        # Get joystick values
        left_joy_x = applyDeadband(controller.lefty(), 0.1) * mult * self.max_speed
        left_joy_y = applyDeadband(controller.leftx(), 0.1) * mult * self.max_speed

        # Get the current POV from the controller
        if controller.pov() >= 0:
            left_joy_x = -controller.pov_x() * mult * self.max_speed
            left_joy_y = -controller.pov_y() * mult * self.max_speed

        # calculate max angular speed based on max_speed (cool math here)
        omega = self.max_speed / math.dist((0, 0), (self.offset_x, self.offset_y))
        right_joy_x = applyDeadband(controller.rightx(), 0.1) * mult * omega

        if left_joy_x != 0 or left_joy_y != 0 or right_joy_x != 0:
            self.swerve_drive.drive(
                -left_joy_y,
                left_joy_x,
                -right_joy_x,
                self.max_speed,
                not controller.leftbumper(),
                self.period,
            )

        if controller.startbutton():
            self.navX.reset()
            self.navX.setAngleAdjustment(-90)
            self.navx_alert.set(True)

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

from components.swerve_wheel import SwerveWheel
import math
import navx
from wpilib import SmartDashboard
from wpimath.kinematics import SwerveDrive4Kinematics
from wpimath.geometry import Translation2d
from wpimath.kinematics import ChassisSpeeds
from wpiutil import Sendable, SendableBuilder
from magicbot import will_reset_to


class SwerveDrive(Sendable):
    offset_x: float
    offset_y: float
    drive_gear_ratio: float
    wheel_radius: float
    front_left: SwerveWheel
    front_right: SwerveWheel
    rear_left: SwerveWheel
    rear_right: SwerveWheel
    navX: navx.AHRS

    stopped = will_reset_to(True)
    translationX = will_reset_to(0)
    translationY = will_reset_to(0)
    rotationX = will_reset_to(0)

    def __init__(self) -> None:
        Sendable.__init__(self)
        self.max_speed = 3.0

    def setup(self) -> None:
        """
        This function is automatically called after the components have
        been injected.
        """
        # Kinematics
        self.front_left_pose = Translation2d(self.offset_x, self.offset_y)
        self.front_right_pose = Translation2d(self.offset_x, -self.offset_y)
        self.rear_left_pose = Translation2d(-self.offset_x, self.offset_y)
        self.rear_right_pose = Translation2d(-self.offset_x, -self.offset_y)
        self.kinematics = SwerveDrive4Kinematics(
            self.front_left_pose,
            self.front_right_pose,
            self.rear_left_pose,
            self.rear_right_pose,
        )
        self.swerve_module_states = self.kinematics.toSwerveModuleStates(
            ChassisSpeeds(0, 0, 0)
        )
        SmartDashboard.putData("Swerve Drive", self)

    def initSendable(self, builder: SendableBuilder) -> None:
        builder.setSmartDashboardType("SwerveDrive")
        builder.addDoubleProperty(
            "Front Left Velocity",
            lambda: self.swerve_module_states[0].speed,
            lambda _: None,
        )
        builder.addDoubleProperty(
            "Front Left Angle",
            lambda: self.swerve_module_states[0].angle.radians(),
            lambda _: None,
        )
        builder.addDoubleProperty(
            "Front Right Velocity",
            lambda: self.swerve_module_states[1].speed,
            lambda _: None,
        )
        builder.addDoubleProperty(
            "Front Right Angle",
            lambda: self.swerve_module_states[1].angle.radians(),
            lambda _: None,
        )
        builder.addDoubleProperty(
            "Back Left Velocity",
            lambda: self.swerve_module_states[2].speed,
            lambda _: None,
        )
        builder.addDoubleProperty(
            "Back Left Angle",
            lambda: self.swerve_module_states[2].angle.radians(),
            lambda _: None,
        )
        builder.addDoubleProperty(
            "Back Right Velocity",
            lambda: self.swerve_module_states[3].speed,
            lambda _: None,
        )
        builder.addDoubleProperty(
            "Back Right Angle",
            lambda: self.swerve_module_states[3].angle.radians(),
            lambda _: None,
        )

    """
    CONTROL METHODS

    These essentially set up variables and info before execute is ran 
    (like updating translationX from 0 -> 1)
    """

    def drive(
        self,
        translationX: float,
        translationY: float,
        rotationX: float,
        max_speed: float,
    ):
        self.translationX = translationX
        self.translationY = translationY
        self.rotationX = rotationX
        self.max_speed = max_speed
        self.stopped = False

    def reset_gyro(self) -> None:
        self.navX.reset()

    """
    EXECUTE
    This is ran every "tick" of the robot. This is where we update all 
    the wheels speed and direction.
    """

    def execute(self) -> None:
        if self.stopped:
            return

        self.swerve_module_states = self.kinematics.toSwerveModuleStates(
            ChassisSpeeds.discretize(
                (
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                        self.translationX,
                        self.translationY,
                        self.rotationX,
                        self.navX.getRotation2d(),
                    )
                ),
                0.02,
            )
        )
        self.swerve_module_states = SwerveDrive4Kinematics.desaturateWheelSpeeds(
            self.swerve_module_states,
            self.max_speed,
        )
        self.front_left.setDesiredState(self.swerve_module_states[0])
        self.front_right.setDesiredState(self.swerve_module_states[1])
        self.rear_left.setDesiredState(self.swerve_module_states[2])
        self.rear_right.setDesiredState(self.swerve_module_states[3])

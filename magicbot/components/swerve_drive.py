import math

import navx
from wpilib import SmartDashboard
from wpimath.kinematics import SwerveDrive4Kinematics
from wpimath.geometry import Translation2d, Rotation2d, Pose2d
from wpimath.kinematics import ChassisSpeeds, SwerveModulePosition
from wpimath.estimator import SwerveDrive4PoseEstimator
from wpiutil import Sendable, SendableBuilder
from magicbot import will_reset_to, feedback

from components.swerve_wheel import SwerveWheel
from util.alerts import Alert, AlertType


class SwerveDrive(Sendable):
    offset_x: float
    offset_y: float
    drive_gear_ratio: float
    wheel_radius: float
    max_speed: float
    front_left: SwerveWheel
    front_right: SwerveWheel
    rear_left: SwerveWheel
    rear_right: SwerveWheel
    navX: navx.AHRS

    translationX = will_reset_to(0)
    translationY = will_reset_to(0)
    rotationX = will_reset_to(0)
    field_relative = will_reset_to(True)

    def __init__(self) -> None:
        Sendable.__init__(self)
        self.period = 0.02

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
        self.chassis_speeds = ChassisSpeeds()
        self.still_states = self.kinematics.toSwerveModuleStates(self.chassis_speeds)
        self.swerve_module_states = self.still_states
        SmartDashboard.putData("Swerve Drive", self)

        self.pose_estimator = SwerveDrive4PoseEstimator(
            self.kinematics,
            Rotation2d(),
            (
                SwerveModulePosition(),
                SwerveModulePosition(),
                SwerveModulePosition(),
                SwerveModulePosition(),
            ),
            Pose2d(x=0, y=0, angle=0),
        )

        SmartDashboard.putData("Gyro", self.navX)
        self.navx_alert = Alert(
            "NavX heading has been reset", AlertType.INFO, timeout=3.0
        )

    def initSendable(self, builder: SendableBuilder) -> None:
        builder.setSmartDashboardType("SwerveDrive")
        builder.addDoubleProperty(
            "Robot Angle",
            # Rotate to match field widget
            lambda: self.navX.getRotation2d().degrees(),
            lambda _: None,
        )
        builder.addDoubleProperty(
            "Front Left Velocity",
            lambda: self.swerve_module_states[0].speed * 5 / self.max_speed,
            lambda _: None,
        )
        builder.addDoubleProperty(
            "Front Left Angle",
            lambda: self.swerve_module_states[0].angle.degrees(),
            lambda _: None,
        )
        builder.addDoubleProperty(
            "Front Right Velocity",
            lambda: self.swerve_module_states[1].speed * 5 / self.max_speed,
            lambda _: None,
        )
        builder.addDoubleProperty(
            "Front Right Angle",
            lambda: self.swerve_module_states[1].angle.degrees(),
            lambda _: None,
        )
        builder.addDoubleProperty(
            "Back Left Velocity",
            lambda: self.swerve_module_states[2].speed * 5 / self.max_speed,
            lambda _: None,
        )
        builder.addDoubleProperty(
            "Back Left Angle",
            lambda: self.swerve_module_states[2].angle.degrees(),
            lambda _: None,
        )
        builder.addDoubleProperty(
            "Back Right Velocity",
            lambda: self.swerve_module_states[3].speed * 5 / self.max_speed,
            lambda _: None,
        )
        builder.addDoubleProperty(
            "Back Right Angle",
            lambda: self.swerve_module_states[3].angle.degrees(),
            lambda _: None,
        )

    def get_estimated_pose(self) -> Pose2d:
        return self.pose_estimator.getEstimatedPosition()

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
        field_relative: bool,
        period: float,
    ):
        self.translationX = translationX
        self.translationY = translationY
        self.rotationX = rotationX
        self.period = period
        self.field_relative = field_relative

    def reset_gyro(self) -> None:
        self.navX.reset()
        self.navX.setAngleAdjustment(-90)
        self.navx_alert.set(True)

    def add_vision_measurement(self, pose, timestamp):
        self.pose_estimator.addVisionMeasurement(pose, timestamp)

    def sendAdvantageScopeData(self):
        """Put swerve module setpoints and measurements to NT.
        This is used mainly for AdvantageScope's swerve tab"""
        swerve_setpoints = []
        for state in self.swerve_module_states:
            swerve_setpoints += [state.angle.degrees(), state.speed]
        SmartDashboard.putNumberArray("Swerve Setpoints", swerve_setpoints)
        swerve_measurements = []
        swerve_measurements += self.front_left.getMeasuredState()
        swerve_measurements += self.front_right.getMeasuredState()
        swerve_measurements += self.rear_left.getMeasuredState()
        swerve_measurements += self.rear_right.getMeasuredState()
        SmartDashboard.putNumberArray("Swerve Measurements", swerve_measurements)

    """
    EXECUTE
    This is ran every "tick" of the robot. This is where we update all 
    the wheels speed and direction.
    """

    def on_enable(self):
        self.navX.reset()
        self.navX.setAngleAdjustment(-90)

    def execute(self) -> None:
        self.sendAdvantageScopeData()
        self.pose_estimator.update(
            Rotation2d(-self.navX.getAngle() / 180 * math.pi),
            (
                self.front_left.getPosition(),
                self.front_right.getPosition(),
                self.rear_left.getPosition(),
                self.rear_right.getPosition(),
            ),
        )

        if self.translationX == self.translationY == self.rotationX == 0:
            # below line is only to keep NT updated
            self.swerve_module_states = self.still_states
            self.chassis_speeds = ChassisSpeeds()
            return

        self.chassis_speeds = ChassisSpeeds.discretize(
            (
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    self.translationX,
                    self.translationY,
                    self.rotationX,
                    self.navX.getRotation2d(),
                )
                if self.field_relative
                else ChassisSpeeds.fromFieldRelativeSpeeds(
                    self.translationX,
                    self.translationY,
                    self.rotationX,
                    Rotation2d(math.pi / 2),
                )
            ),
            self.period,
        )
        self.swerve_module_states = self.kinematics.toSwerveModuleStates(
            self.chassis_speeds
        )
        self.swerve_module_states = SwerveDrive4Kinematics.desaturateWheelSpeeds(
            self.swerve_module_states,
            self.max_speed,
        )
        self.front_left.setDesiredState(self.swerve_module_states[0])
        self.front_right.setDesiredState(self.swerve_module_states[1])
        self.rear_left.setDesiredState(self.swerve_module_states[2])
        self.rear_right.setDesiredState(self.swerve_module_states[3])

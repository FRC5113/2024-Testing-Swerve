from components.swerve_wheel import SwerveWheel
import math
import navx
from wpilib import SmartDashboard, SendableChooser
from wpimath.kinematics import SwerveDrive4Kinematics
from wpimath.geometry import Translation2d, Rotation2d, Pose2d
from wpimath.kinematics import ChassisSpeeds, SwerveDrive4Odometry, SwerveModulePosition
from wpiutil import Sendable, SendableBuilder
from magicbot import will_reset_to
from wpilib import DriverStation
from pathplannerlib.auto import AutoBuilder, HolonomicPathFollowerConfig, ReplanningConfig
from pathplannerlib.commands import PathfindThenFollowPathHolonomic
from pathplannerlib.controller import PPHolonomicDriveController
from pathplannerlib.config import HolonomicPathFollowerConfig
from pathplannerlib.logging import PathPlannerLogging
from pathplannerlib.path import PathConstraints, PathPlannerPath
from pathplannerlib.controller import PIDConstants


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
    field_relative = will_reset_to(True)


    # PathPlanner Config
    path_follower_config = HolonomicPathFollowerConfig(
        # Holonomic-specific config
        PIDConstants( # PID for translation
            Constants.PathPlanner.k_translation_p,
            Constants.PathPlanner.k_translation_i,
            Constants.PathPlanner.k_translation_d,
            Constants.PathPlanner.k_translation_i_zone
        ), 
        PIDConstants( # PID for rotation
            Constants.PathPlanner.k_rotation_p,
            Constants.PathPlanner.k_rotation_i,
            Constants.PathPlanner.k_rotation_d,
            Constants.PathPlanner.k_rotation_i_zone
        ), 
        Constants.Drivetrain.k_max_attainable_speed, # Max module speed (matches the one in PathPlanner)
        Constants.Drivetrain.k_drive_base_radius, # Distance from center of the robot to a swerve module
        ReplanningConfig() # Replanning Config (check the docs, this is hard to explain)
    )

    def __init__(self) -> None:
        Sendable.__init__(self)
        self.max_speed = 1.0
        self.period = 0.02

        # Configure PathPlanner
        AutoBuilder.configureHolonomic(
            self.odometry.getPose,
            lambda pose: self.navX.reset(),
            self.kinematics.toChassisSpeeds(ChassisSpeeds.fromRobotRelativeSpeeds(0, 0, 0)),
            lambda speeds: self.drive_robot_centric(speeds),
            self.path_follower_config,
            lambda: DriverStation.getAlliance() == DriverStation.Alliance.kRed, # "Hey Caden, when do we flip the path?"
            self # "Yes, this is the drivetrain. Why would I configure an AutoBuilder for an intake, pathplannerlib?"
        )

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

        self.odometry = SwerveDrive4Odometry(
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


    def onRedAlliance(self):
        # Returns boolean that equals true if we are on the Red Alliance
        return DriverStation.getAlliance() == DriverStation.Alliance.kRed

    def shouldFlipPath(self):
        # Boolean supplier that controls when the path will be mirrored for the red alliance
        # This will flip the path being followed to the red side of the field.
        # THE ORIGIN WILL REMAIN ON THE BLUE SIDE
        return self.onRedAlliance()

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
        field_relative: bool,
        period: float,
    ):
        self.translationX = translationX
        self.translationY = translationY
        self.rotationX = rotationX
        self.max_speed = max_speed
        self.period = period
        self.field_relative = field_relative
        self.stopped = False

    """
    EXECUTE
    This is ran every "tick" of the robot. This is where we update all 
    the wheels speed and direction.
    """

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

    def execute(self) -> None:
        self.sendAdvantageScopeData()

        if self.stopped:
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

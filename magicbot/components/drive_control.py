import math

from navx import AHRS
from wpimath.controller import HolonomicDriveController
import wpimath
from wpimath.trajectory import TrapezoidProfileRadians
import wpimath.trajectory
from util.smart_preference import SmartProfile
from wpimath.geometry import Rotation2d, Pose2d
from wpimath.kinematics import SwerveDrive4Kinematics
from wpimath.kinematics import ChassisSpeeds
from magicbot import StateMachine, state, default_state, will_reset_to

from components.odometry import Odometry
from components.swerve_drive import SwerveDrive


class DriveControl(StateMachine):
    """States:
    idle: robot not moving, in break mode
    driving_manual: driving with hid input
    driving_to_pose: driving to set pose (with holonomic controller)
    """

    odometry: Odometry
    swerve_drive: SwerveDrive

    holonomic_x_profile: SmartProfile
    holonomic_y_profile: SmartProfile
    holonomic_theta_profile: SmartProfile
    navX: AHRS

    translationX = will_reset_to(0)
    translationY = will_reset_to(0)
    rotationX = will_reset_to(0)
    field_relative = will_reset_to(True)
    period = 0.02

    # def setup(self) -> None:
    #     self.x_controller = self.holonomic_x_profile.create_controller(
    #         "holonomic_x"
    #     )
    #     self.y_controller = self.holonomic_y_profile.create_controller(
    #         "holonomic_y"
    #     )
    #     self.theta_controller = self.holonomic_theta_profile.create_controller(
    #         "holonomic_theta"
    #     )
    #     self.holonomic_controller = HolonomicDriveController(
    #         self.x_controller,
    #         self.y_controller,
    #         self.theta_controller
    #     )

    def drive_manual(
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

    @state(first=True)
    def idle(self):
        if not (self.translationX == self.translationY == self.rotationX == 0):
            self.next_state_now("driving_manual")
            return
        self.swerve_drive.set_speeds(ChassisSpeeds())

    @state
    def driving_manual(self):
        if self.translationX == self.translationY == self.rotationX == 0:
            self.next_state_now("idle")
            return
        chassis_speeds = ChassisSpeeds.discretize(
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
                    Rotation2d(),
                )
            ),
            self.period,
        )
        self.swerve_drive.set_speeds(chassis_speeds)

    @state
    def driving_to_pose(self):
        pass

    # def adjustedSpeeds(self):
    #     adjustedSpeeds = self.controller.calculate(
    #     Odometry.get_estimated_pose, wpimath.trajectory.Trajectory.sample, SwerveDrive.navX.getRotation2d
    #     )

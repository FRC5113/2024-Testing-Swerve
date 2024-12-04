import math

from navx import AHRS
from wpimath.controller import HolonomicDriveController
import wpimath
from wpimath.trajectory import TrapezoidProfileRadians
import wpimath.trajectory
from util.smart_preference import SmartProfile
from wpimath.geometry import Rotation2d,Pose2d
from wpimath.kinematics import SwerveDrive4Kinematics
from wpimath.kinematics import ChassisSpeeds
from magicbot import StateMachine, state, default_state, will_reset_to

from components.odometry import Odometry
from components.swerve_drive import SwerveDrive


class DriveControl(StateMachine):
    """States:
    disabled
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

    def setup(self) -> None:
        self.x_controller = self.holonomic_x_profile.create_controller(
            "holonomic_x"
        )
        self.y_controller = self.holonomic_y_profile.create_controller(
            "holonomic_y"
        )
        self.theta_controller = self.holonomic_theta_profile.create_controller(
            "holonomic_theta"
        )
        self.holonomic_controller = HolonomicDriveController(
            self.x_controller,
            self.y_controller,
            self.theta_controller
        )

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

    @default_state
    def disabled(self):
        self.swerve_drive.set_speeds(ChassisSpeeds())

    @state(first=True)
    def driving_manual(self):
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

    def adjustedSpeeds(self):
        adjustedSpeeds = self.controller.calculate(
        Odometry.get_estimated_pose, wpimath.trajectory.Trajectory.sample, SwerveDrive.navX.getRotation2d
        ) 


        
# class drivecontrol:
#     offset_x: float
#     offset_y: float
#     drive_gear_ratio: float
#     wheel_radius: float
#     max_speed: float

    

#     def execute(self) -> None:
#         SwerveDrive.sendAdvantageScopeData()
#         SwerveDrive.pose_estimator.update(
#             Rotation2d(-SwerveDrive.navX.getAngle() / 180 * math.pi),
#             (
#                 SwerveDrive.front_left.getPosition(),
#                 SwerveDrive.front_right.getPosition(),
#                 SwerveDrive.rear_left.getPosition(),
#                 SwerveDrive.rear_right.getPosition(),
#             ),
#         )

#         if SwerveDrive.translationX == SwerveDrive.translationY == SwerveDrive.rotationX == 0:
#             # below line is only to keep NT updated
#             self.swerve_module_states = SwerveDrive.still_states
#             self.chassis_speeds = ChassisSpeeds()
#             return

#         self.chassis_speeds = ChassisSpeeds.discretize(
#             (
#                 ChassisSpeeds.fromFieldRelativeSpeeds(
#                     SwerveDrive.translationX,
#                     SwerveDrive.translationY,
#                     SwerveDrive.rotationX,
#                     SwerveDrive.navX.getRotation2d(),
#                 )
#                 if SwerveDrive.field_relative
#                 else ChassisSpeeds.fromFieldRelativeSpeeds(
#                     SwerveDrive.translationX,
#                     SwerveDrive.translationY,
#                     SwerveDrive.rotationX,
#                     Rotation2d(),
#                 )
#             ),
#             SwerveDrive.period,
#         )
#         self.swerve_module_states = SwerveDrive.kinematics.toSwerveModuleStates(
#             Holocontroller.adjustedSpeeds
#         )
        
#         self.swerve_module_states = SwerveDrive4Kinematics.desaturateWheelSpeeds(
#             self.swerve_module_states,
#             self.max_speed,
#         )
#         SwerveDrive.front_left.setDesiredState(self.swerve_module_states[0])
#         SwerveDrive.front_right.setDesiredState(self.swerve_module_states[1])
#         SwerveDrive.rear_left.setDesiredState(self.swerve_module_states[2])
#         SwerveDrive.rear_right.setDesiredState(self.swerve_module_states[3])
    
    
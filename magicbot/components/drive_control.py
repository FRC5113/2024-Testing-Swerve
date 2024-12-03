import math

from wpimath.controller import HolonomicDriveController
import wpimath
from wpimath.trajectory import TrapezoidProfileRadians
import wpimath.trajectory
from util.smart_preference import SmartProfile
from wpimath.geometry import Rotation2d,Pose2d
from components.odometry import Odometry
from swerve_drive import SwerveDrive
from components.swerve_wheel import SwerveWheel
import navx
from wpimath.kinematics import SwerveDrive4Kinematics
from wpimath.kinematics import ChassisSpeeds


class DriveControl():
    holonomic_x_profile: SmartProfile
    holonomic_y_profile: SmartProfile
    holonomic_theta_profile: SmartProfile

    def __init__(self) -> None:
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

    def adjustedSpeeds(self):
        adjustedSpeeds = self.controller.calculate(
        Odometry.get_estimated_pose, wpimath.trajectory.Trajectory.sample, SwerveDrive.navX.getRotation2d
        ) 


        
class drivecontrol:
    offset_x: float
    offset_y: float
    drive_gear_ratio: float
    wheel_radius: float
    max_speed: float

    

    def execute(self) -> None:
        SwerveDrive.sendAdvantageScopeData()
        SwerveDrive.pose_estimator.update(
            Rotation2d(-SwerveDrive.navX.getAngle() / 180 * math.pi),
            (
                SwerveDrive.front_left.getPosition(),
                SwerveDrive.front_right.getPosition(),
                SwerveDrive.rear_left.getPosition(),
                SwerveDrive.rear_right.getPosition(),
            ),
        )

        if SwerveDrive.translationX == SwerveDrive.translationY == SwerveDrive.rotationX == 0:
            # below line is only to keep NT updated
            self.swerve_module_states = SwerveDrive.still_states
            self.chassis_speeds = ChassisSpeeds()
            return

        self.chassis_speeds = ChassisSpeeds.discretize(
            (
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    SwerveDrive.translationX,
                    SwerveDrive.translationY,
                    SwerveDrive.rotationX,
                    SwerveDrive.navX.getRotation2d(),
                )
                if SwerveDrive.field_relative
                else ChassisSpeeds.fromFieldRelativeSpeeds(
                    SwerveDrive.translationX,
                    SwerveDrive.translationY,
                    SwerveDrive.rotationX,
                    Rotation2d(),
                )
            ),
            SwerveDrive.period,
        )
        self.swerve_module_states = SwerveDrive.kinematics.toSwerveModuleStates(
            Holocontroller.adjustedSpeeds
        )
        
        self.swerve_module_states = SwerveDrive4Kinematics.desaturateWheelSpeeds(
            self.swerve_module_states,
            self.max_speed,
        )
        SwerveDrive.front_left.setDesiredState(self.swerve_module_states[0])
        SwerveDrive.front_right.setDesiredState(self.swerve_module_states[1])
        SwerveDrive.rear_left.setDesiredState(self.swerve_module_states[2])
        SwerveDrive.rear_right.setDesiredState(self.swerve_module_states[3])
    
    
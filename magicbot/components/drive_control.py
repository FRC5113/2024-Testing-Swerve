from wpimath.controller import (
   HolonomicDriveController,
   PIDController,
   ProfiledPIDControllerRadians
)
import wpimath
from wpimath.trajectory import TrapezoidProfileRadians
import wpimath.trajectory
from util.smart_preference import SmartProfile
from wpimath.geometry import Rotation2d,Pose2d
from components.odometry import Odometry
from swerve_drive import SwerveDrive
import math
from components.swerve_wheel import SwerveWheel
import navx
from wpimath.kinematics import SwerveDrive4Kinematics
from wpimath.kinematics import ChassisSpeeds


class Holocontroller():
    transx_profile = SmartProfile
    transy_profile = SmartProfile
    

    def __init__(self) -> None:
        self.transx_controller = self.transx_profile.create_controller(
            "Transx"
        )
        self.transy_controller = self.transy_profile.create_controller(
            "TransY"
        )
        self.controller = HolonomicDriveController(
        self.transx_controller,
        self.transy_controller,
        ProfiledPIDControllerRadians(
            1, 0, 0, TrapezoidProfileRadians.Constraints(6.28, 3.14)
        ),
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
    
    
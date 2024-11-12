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
from components.odometry import SwerveDrive,Odometry




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
        wpimath.trajectory.Trajectory.sample
    
    
from wpilib import Timer, Field2d, SmartDashboard
from wpimath.geometry import Pose2d, Translation2d
from navx import AHRS
from robotpy_apriltag import AprilTagFieldLayout

from components.swerve_drive import SwerveDrive
from util.camera import LemonCamera


class Odometry:
    camera: LemonCamera
    navX: AHRS
    field_layout: AprilTagFieldLayout
    swerve_drive: SwerveDrive

    def setup(self):
        self.estimated_field = Field2d()
        self.tag_object = self.estimated_field.getObject("tag")
        SmartDashboard.putData("Estimated Field", self.estimated_field)

    def get_estimated_pose(self) -> None | Pose2d:
        if not self.camera.has_targets():
            return
        rt = self.camera.get_pose().translation()
        theta = self.navX.getRotation2d()
        rt = rt.rotateBy(theta)
        tag_pose = self.field_layout.getTagPose(self.camera.get_best_id())
        ot = Translation2d(tag_pose.x, tag_pose.y)
        return Pose2d(ot - rt, theta)

    def execute(self):
        self.camera.update()
        if self.camera.has_targets():
            self.swerve_drive.add_vision_measurement(
                self.get_estimated_pose(), Timer.getFPGATimestamp()
            )
            self.tag_object.setPose(
                self.field_layout.getTagPose(self.camera.get_best_id()).toPose2d()
            )
        else:
            self.tag_object.setPose(Pose2d())
        self.estimated_field.setRobotPose(self.swerve_drive.get_estimated_pose())

from wpilib import Timer, FieldObject2d
from wpimath.geometry import Pose2d, Translation2d
from navx import AHRS
from robotpy_apriltag import AprilTagFieldLayout

from components.swerve_drive import SwerveDrive
from util.wrappers import LemonCamera


class Vision:
    camera: LemonCamera
    navX: AHRS
    field_layout: AprilTagFieldLayout
    swerve_drive: SwerveDrive
    tag_object: FieldObject2d

    def get_estimated_pose(self) -> None | Pose2d:
        if not self.camera.hasTargets():
            return
        rt = self.camera.getAdjustedTranslation()
        theta = self.navX.getRotation2d()
        rt = rt.rotateBy(theta)
        tag_pose = self.field_layout.getTagPose(self.camera.getId())
        ot = Translation2d(tag_pose.x, tag_pose.y)
        return Pose2d(ot - rt, theta)

    def execute(self):
        self.camera.update()
        if self.camera.hasTargets():
            self.swerve_drive.add_vision_measurement(
                self.get_estimated_pose(), Timer.getFPGATimestamp()
            )
            self.tag_object.setPose(
                self.field_layout.getTagPose(self.camera.getId()).toPose2d()
            )
        else:
            self.tag_object.setPose(Pose2d())

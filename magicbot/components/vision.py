from wpimath.geometry import Pose2d
from navx import AHRS

from util.wrappers import LemonCamera


class Vision:
    camera: LemonCamera
    navX: AHRS

    def get_estimated_pose(self) -> None | Pose2d:
        if not self.camera.hasTargets():
            return
        rt = self.camera.getAdjustedTranslation()
        theta = self.navX.getAngle()
        rt = rt.rotateBy(theta)

    def execute(self):
        self.camera.update()
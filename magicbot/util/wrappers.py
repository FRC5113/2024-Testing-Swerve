import math

from wpilib.interfaces import MotorController
from wpimath.filter import MedianFilter
from wpimath.geometry import Translation2d, Rotation2d, Pose2d
import phoenix6
from photonlibpy.photonCamera import PhotonCamera
from robotpy_apriltag import AprilTagFieldLayout


class LemonTalonFX(phoenix6.hardware.TalonFX, MotorController):
    """Wrapper for the phoenix6 TalonFX that implements
    the wpilib MotorController interface, making it possible
    to use TalonFX controllers in, for example, MotorControllerGroup
    and DifferentialDrive
    """

    def __init__(self, device_id: int, canbus: str = "", enable_foc: bool = False):
        phoenix6.hardware.TalonFX.__init__(self, device_id, canbus=canbus)
        MotorController.__init__(self)
        self.config = phoenix6.configs.TalonFXConfiguration()
        self.duty_cycle_out = phoenix6.controls.DutyCycleOut(0, enable_foc=enable_foc)
        self.voltage_out = phoenix6.controls.VoltageOut(0, enable_foc=enable_foc)
        self.is_disabled = False

    def disable(self):
        self.stopMotor()
        self.is_disabled = True

    def get(self) -> float:
        return self.duty_cycle_out.output

    def getInverted(self) -> bool:
        return (
            self.config.motor_output.inverted
            == phoenix6.signals.InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        )

    def set(self, speed: float):
        if not self.is_disabled:
            self.duty_cycle_out.output = speed
            self.set_control(self.duty_cycle_out)

    def setIdleMode(self, mode: phoenix6.signals.NeutralModeValue):
        """Set the idle mode setting

        Arguments:
        mode -- Idle mode (coast or brake)
        """
        self.config.motor_output.neutral_mode = mode
        self.configurator.apply(self.config)

    def setInverted(self, isInverted: bool):
        if isInverted:
            self.config.motor_output.inverted = (
                phoenix6.signals.InvertedValue.CLOCKWISE_POSITIVE
            )
        else:
            self.config.motor_output.inverted = (
                phoenix6.signals.InvertedValue.COUNTER_CLOCKWISE_POSITIVE
            )
        self.configurator.apply(self.config)

    def setVoltage(self, volts: float):
        if not self.is_disabled:
            self.voltage_out.output = volts
            self.set_control(self.voltage_out)

    def stopMotor(self):
        self.set(0)


class LemonCamera(PhotonCamera):
    """Wrapper for photonlibpy PhotonCamera that adds the following features:
    1. Filters on x, y, and z values
    2. Protection against brief lapses in tag detection
    3. Heading of tag relative to camera and relative to robot
    4. Retrieval of tags by ID"""

    def __init__(
        self,
        camera_name: str,
        rc: Translation2d,
        tilt: float = 0,
        filter_window: int = 10,
        sought_ids: list[int] = None,
    ):
        """Parameters:
        camera_name -- name of camera in PhotonVision
        rc -- vector from center of robot to camera. z-coord ignored
        tilt -- pitch of camera in degrees. Tilt up is positive
        filter_window -- size of window on MedianFilters. Also the
            number of ticks until a tag is considered lost. Higher
            values yield less noisy and spotty data but with less
            accuracy and precision and higher latency.
        sought_ids -- tag ids the camera will look for. Tags that it
            finds with non-sought ids will be ignored.
        """
        PhotonCamera.__init__(self, camera_name)
        self.rc = rc
        self.tilt = tilt
        self.filter_window = filter_window
        self.x = 0
        self.y = 0
        self.z = 0
        self.x_filter = MedianFilter(self.filter_window)
        self.y_filter = MedianFilter(self.filter_window)
        self.z_filter = MedianFilter(self.filter_window)
        self.latency = 0
        self.drought = self.filter_window
        self.sought_ids = sought_ids

    def update(self) -> None:
        """Call this every loop"""
        result = self.getLatestResult()
        if result.hasTargets():
            self.drought = 0
            potential_targets = result.getTargets()
            if self.sought_ids is not None:
                potential_targets = list(
                    filter(
                        lambda t: t.getFiducialId() in self.sought_ids, potential_targets
                    )
                )
            if len(potential_targets) == 0:
                self.drought += 1
                return
            target = min(potential_targets, key=lambda t: t.getPoseAmbiguity())
            transform = target.getBestCameraToTarget()
            self.id = target.getFiducialId()
            self.latency = result.getLatencyMillis() / 1000
            u = transform.X()
            v = transform.Y()
            w = transform.Z()
            theta = self.tilt * math.pi / 180
            self.x = self.x_filter.calculate(u * math.cos(theta) - w * math.sin(theta))
            self.y = self.y_filter.calculate(v)
            self.z = self.z_filter.calculate(u * math.sin(theta) + w * math.cos(theta))
        else:
            self.drought += 1

    def _check_drought(func):
        def inner(self: LemonCamera):
            if self.hasTargets():
                return func(self)
            else:
                return None
        return inner

    def hasTargets(self) -> bool:
        return self.drought < self.filter_window

    @_check_drought
    def getX(self) -> float | None:
        return self.x

    @_check_drought
    def getY(self) -> float | None:
        return self.y

    @_check_drought
    def getZ(self) -> float | None:
        return self.z

    @_check_drought
    def getId(self) -> int | None:
        return self.id

    @_check_drought
    def getLatency(self) -> float | None:
        return self.latency

    @_check_drought
    def getHeading(self) -> Rotation2d | None:
        """returns angle that robot must turn to face tag"""
        return Rotation2d(math.atan2(-self.y, self.x))

    @_check_drought
    def getAdjustedHeading(self) -> float | None:
        """Returns the angle from the center of the robot to the tag"""
        ct = Translation2d(self.x, self.y)
        rt = self.rc + ct
        return Rotation2d(math.atan2(rt.y, rt.x))
    
    @_check_drought
    def getAdjustedTranslation(self) -> Translation2d | None:
        """Returns the translation from the center of the robot to the tag
        (Relative to the robot)
        """
        ct = Translation2d(self.x, self.y)
        rt = self.rc + ct
        return Translation2d(rt.x, rt.y)


    def setSoughtIds(self, sought_ids):
        self.sought_ids = sought_ids


class SimLemonCamera(LemonCamera):
    def __init__(self, field_layout: AprilTagFieldLayout, fov, rc, tilt, filter_window=10, sought_ids=None):
        LemonCamera.__init__(self, "Sim", rc, tilt, filter_window, sought_ids)
        self.field_layout = field_layout
        self.fov = fov

    def update(self, pose: Pose2d):
        result = []
        for tag in self.field_layout.getTags():
            pass
        # TODO: finish everything lol
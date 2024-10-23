import math

from wpilib import (
    PS5Controller,
    XboxController,
    DriverStation,
)
from wpilib.interfaces import MotorController
from wpimath.filter import MedianFilter
from wpimath.geometry import Translation2d, Rotation2d, Pose2d
import phoenix6
from photonlibpy.photonTrackedTarget import PhotonTrackedTarget
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

class SmartController:
    """Wrapper class for the XboxController and PS5Controller classes that allows to use
    both xbox and ps5 controllers automatically
    without chaging the code.
    Must be called in TeleopPeriodic or AutonomousPeriodic to get the values of the controller for example:
    ```
    def teleopPeriodic(self):
                port_number = 0
                smart_controller = SmartController(port_number)
    ```
    """

    def __init__(self, port_number: int):
        if DriverStation.getJoystickIsXbox(port_number):
            self.driver_controller = XboxController(port_number)
        else:
            self.driver_controller = PS5Controller(port_number)

    def _get_button_method(self, method_name: str):
        """Helper method to get the appropriate button method based on controller type."""
        if isinstance(self.driver_controller, XboxController):
            return getattr(self.driver_controller, method_name)()
        elif isinstance(self.driver_controller, PS5Controller):
            ps5_method_mapping = {
                "getLeftBumper": "getL1Button",
                "getRightBumper": "getR1Button",
                "getStartButton": "getOptionsButton",
                "getAButton": "getCrossButton",
                "getBButton": "getCircleButton",
                "getXButton": "getSquareButton",
                "getYButton": "getTriangleButton",
                "getLeftStickButton": "getL3Button",
                "getRightStickButton": "getR3Button",
                "getLeftX": "getLeftX",
                "getLeftY": "getLeftY",
                "getRightX": "getRightX",
                "getRightY": "getRightY",
                "getPOV": "getPOV",
            }
            return getattr(self.driver_controller, ps5_method_mapping[method_name])()

    def leftbumper(self):
        """
        Returns the state of the left bumper button.

        Returns:
            bool: The state of the left bumper button (pressed or not).
        """
        return self._get_button_method("getLeftBumper")

    def rightbumper(self):
        """
        Returns the state of the right bumper button.

        Returns:
            bool: The state of the right bumper button (pressed or not).
        """
        return self._get_button_method("getRightBumper")

    def startbutton(self):
        """
        Returns the state of the start button.

        Returns:
            bool: The state of the start button (pressed or not).
        """
        return self._get_button_method("getStartButton")

    def abutton(self):
        """
        Returns the state of the 'A' button.

        Returns:
            bool: The state of the 'A' button (pressed or not).
        """
        return self._get_button_method("getAButton")

    def bbutton(self):
        """
        Returns the state of the 'B' button.

        Returns:
            bool: The state of the 'B' button (pressed or not).
        """
        return self._get_button_method("getBButton")

    def xbutton(self):
        """
        Returns the state of the 'X' button.

        Returns:
            bool: The state of the 'X' button (pressed or not).
        """
        return self._get_button_method("getXButton")

    def ybutton(self):
        """
        Returns the state of the 'Y' button.

        Returns:
            bool: The state of the 'Y' button (pressed or not).
        """
        return self._get_button_method("getYButton")

    def lstickbutton(self):
        """
        Returns the state of the left stick button.

        Returns:
            bool: The state of the left stick button (pressed or not).
        """
        return self._get_button_method("getLeftStickButton")

    def rstickbutton(self):
        """
        Returns the state of the right stick button.

        Returns:
            bool: The state of the right stick button (pressed or not).
        """
        return self._get_button_method("getRightStickButton")

    def leftx(self) -> float:
        """
        Returns the X-axis value of the left joystick.

        Returns:
            float: The X-axis value of the left joystick, ranging from -1.0 to 1.0.
        """
        return float(self._get_button_method("getLeftX"))

    def lefty(self) -> float:
        """
        Returns the Y-axis value of the left joystick.

        Returns:
            float: The Y-axis value of the left joystick, ranging from -1.0 to 1.0.
        """
        return float(self._get_button_method("getLeftY"))

    def rightx(self) -> float:
        """
        Returns the X-axis value of the right joystick.

        Returns:
            float: The X-axis value of the right joystick, ranging from -1.0 to 1.0.
        """
        return float(self._get_button_method("getRightX"))

    def righty(self) -> float:
        """
        Returns the Y-axis value of the right joystick.

        Returns:
            float: The Y-axis value of the right joystick, ranging from -1.0 to 1.0.
        """
        return float(self._get_button_method("getRightY"))

    def pov(self) -> int:
        """
        Returns the Point of View (POV) value as an integer.

        Returns:
            int: The current POV value. Returns -1 if no POV is pressed.
        """
        return int(self._get_button_method("getPOV"))


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
                        lambda t: t.getFiducialId() in self.sought_ids,
                        potential_targets,
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
        def inner(self):
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
    def __init__(
        self,
        field_layout: AprilTagFieldLayout,
        fov: float,
        rc: Translation2d,
        tilt: float,
        filter_window=10,
        sought_ids=None,
    ):
        LemonCamera.__init__(self, "Sim", rc, tilt, filter_window, sought_ids)
        self.field_layout = field_layout
        self.fov = fov

    def update(self, robot_pose: Pose2d):
        best_ambiguity = 0
        best_pose = None
        best_id = -1
        for tag in self.field_layout.getTags():
            tag_pose = tag.pose.toPose2d()
            relative_pose = tag_pose.relativeTo(robot_pose)
            dist = relative_pose.translation().norm()
            # calculate estimated tag ambiguity based on distance and angle
            ambiguity = (
                -(tag_pose.rotation() - robot_pose.rotation()).cos() / dist / dist
            )
            # check that robot is facing tag and that tag is turned toward robot
            # NOTE: this assumes camera is at center of robot looking forward
            if (
                abs(relative_pose.translation().angle().degrees()) < self.fov / 2
                and ambiguity > 0
                and ambiguity > best_ambiguity
            ):
                best_ambiguity = ambiguity
                best_pose = relative_pose
                best_id = tag.ID

        if best_pose is not None:
            self.latency = 0.02
            self.id = best_id
            self.x = best_pose.x
            self.y = best_pose.y
            self.z = 0
            self.drought = 0
        else:
            self.drought += 1

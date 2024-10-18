from wpilib import PS5Controller, XboxController, SendableChooser, SmartDashboard,DriverStation
from wpilib.interfaces import MotorController
import phoenix6


class WPI_TalonFX(phoenix6.hardware.TalonFX, MotorController):
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
    def __init__(self, xbox: XboxController, ps5: PS5Controller):
        if DriverStation.getJoystickIsXbox(0):
            self.driver_controller = xbox(0)
        else:
            self.driver_controller = ps5(0)
        if DriverStation.getJoystickIsXbox(1):
            self.driver_controller = xbox(1)
        else:
            self.driver_controller = ps5(1)
    def leftbumper(self):
        if isinstance(self.driver_controller, XboxController):
            return self.driver_controller.getLeftBumper()
        elif isinstance(self.driver_controller, PS5Controller):
            return self.driver_controller.getL1Button()
    def rightbumper(self):
        if isinstance(self.driver_controller, XboxController):
            return self.driver_controller.getRightBumper()
        elif isinstance(self.driver_controller, PS5Controller):
            return self.driver_controller.getR1Button()
    def startbutton(self):
        if isinstance(self.driver_controller, XboxController):
            return self.driver_controller.getStartButton()
        elif isinstance(self.driver_controller, PS5Controller):
            return self.driver_controller.getOptionsButton()
    def abutton(self):
        if isinstance(self.driver_controller, XboxController):
            return self.driver_controller.getAButton()
        elif isinstance(self.driver_controller, PS5Controller):
            return self.driver_controller.getCrossButton()
    def bbutton(self):
        if isinstance(self.driver_controller, XboxController):
            return self.driver_controller.getBButton()
        elif isinstance(self.driver_controller, PS5Controller):
            return self.driver_controller.getCircleButton()
    def xbutton(self):
        if isinstance(self.driver_controller, XboxController):
            return self.driver_controller.getXButton()
        elif isinstance(self.driver_controller, PS5Controller):
            return self.driver_controller.getSquareButton()
    def ybutton(self):
        if isinstance(self.driver_controller, XboxController):
            return self.driver_controller.getYButton()
        elif isinstance(self.driver_controller, PS5Controller):
            return self.driver_controller.getTriangleButton()
    def lstickbutton(self):
        if isinstance(self.driver_controller, XboxController):
            return self.driver_controller.getLeftStickButton()
        elif isinstance(self.driver_controller, PS5Controller):
            return self.driver_controller.getL3Button()
    def rstickbutton(self):
        if isinstance(self.driver_controller, XboxController):
            return self.driver_controller.getRightStickButton()
        elif isinstance(self.driver_controller, PS5Controller):
            return self.driver_controller.getR3Button()
    def leftx(self):
        if isinstance(self.driver_controller, XboxController):
            return self.driver_controller.getLeftX()
        elif isinstance(self.driver_controller, PS5Controller):
            return self.driver_controller.getLeftX()
    def lefty(self):
        if isinstance(self.driver_controller, XboxController):
            return self.driver_controller.getLeftY()
        elif isinstance(self.driver_controller, PS5Controller):
            return self.driver_controller.getLeftY()
    def rightx(self):
        if isinstance(self.driver_controller, XboxController):
            return self.driver_controller.getRightX()
        elif isinstance(self.driver_controller, PS5Controller):
            return self.driver_controller.getRightX()
    def righty(self):  
        if isinstance(self.driver_controller, XboxController):
            return self.driver_controller.getRightY()
        elif isinstance(self.driver_controller, PS5Controller):
            return self.driver_controller.getRightY()
    def pov(self):
        if isinstance(self.driver_controller, XboxController):
            return self.driver_controller.getPOV()
        elif isinstance(self.driver_controller, PS5Controller):
            return self.driver_controller.getPOV()
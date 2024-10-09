from wpilib import PS5Controller,XboxController,SendableChooser,SmartDashboard
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
class EasierControllers:
    def __init__(self):
        # Initialize controller chooser
        self.controller_chooser = SendableChooser()
        self.controller_chooser.setDefaultOption("Playstation",  PS5Controller)
        self.controller_chooser.addOption("Xbox",  XboxController)

        SmartDashboard.putData("Controller", self.controller_chooser)

        # Controller selection
        self.controller = self.controller_chooser.getSelected()

        if self.controller ==  XboxController:
            self.driver_controller =  XboxController(0)
        elif self.controller ==  PS5Controller:
            self.driver_controller =  PS5Controller(0)

        # Initialize button variables
        self.initialize_buttons()

    def initialize_buttons(self):
        """Initialize buttons based on the controller type"""
        if isinstance(self.driver_controller,  XboxController):
            self.leftbumper = self.driver_controller.getLeftBumper()
            self.rightbumper = self.driver_controller.getRightBumper()
            self.startbutton = self.driver_controller.getStartButton()
            self.abutton = self.driver_controller.getAButton()
            self.bbutton = self.driver_controller.getBButton()
            self.xbutton = self.driver_controller.getXButton()
            self.ybutton = self.driver_controller.getYButton()

        elif isinstance(self.driver_controller,  PS5Controller):
            self.leftbumper = self.driver_controller.getL1Button()
            self.rightbumper = self.driver_controller.getR1Button()
            self.startbutton = self.driver_controller.getOptionsButton()
            self.abutton = self.driver_controller.getCrossButton()
            self.bbutton = self.driver_controller.getCircleButton()
            self.xbutton = self.driver_controller.getSquareButton()
            self.ybutton = self.driver_controller.getTriangleButton()
    
    def drivercontroller(self):
        return self.driver_controller

    def get_leftbumper(self):
        return self.leftbumper

    def get_rightbumper(self):
        return self.rightbumper

    def get_startbutton(self):
        return self.startbutton

    def get_abutton(self):
        return self.abutton

    def get_bbutton(self):
        return self.bbutton

    def get_xbutton(self):
        return self.xbutton

    def get_ybutton(self):
        return self.ybutton
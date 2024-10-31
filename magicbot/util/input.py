from wpilib import DriverStation, RobotBase, XboxController, PS5Controller


class LemonInput:
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
        if DriverStation.getJoystickIsXbox(port_number) or RobotBase.isSimulation:
            self.driver_controller = XboxController(port_number)
            self.trigger_fix = 0.0
        else:
            self.driver_controller = PS5Controller(port_number)
            self.trigger_fix = 1.0

    def _get_button_method(self, method_name: str):
        """Helper method to get the appropriate button method based on controller type."""
        if isinstance(self.driver_controller, XboxController):
            return getattr(self.driver_controller, method_name)()
        elif isinstance(self.driver_controller, PS5Controller):
            ps5_method_mapping = {
                "getLeftBumper": "getL1Button",
                "getRightBumper": "getR1Button",
                "getStartButton": "getOptionsButton",
                "getBackButton": "getCreateButton",
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
                "getLeftTriggerAxis": "getL2Axis",
                "getRightTriggerAxis": "getR2Axis",
            }
            return getattr(self.driver_controller, ps5_method_mapping[method_name])()

    def type(self):
        """
        Returns the type of controller.

        Returns:
            str: The type of controller (Xbox or PS5).
        """
        if isinstance(self.driver_controller, XboxController):
            return "Xbox"
        elif isinstance(self.driver_controller, PS5Controller):
            return "PS5"

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

    def backbutton(self):
        """
        Returns the state of the back button.

        Returns:
            bool: The state of the back button (pressed or not).
        """
        return self._get_button_method("getBackButton")

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

    def righttrigger(self) -> float:
        """
        Returns the state of the right trigger button.

        Returns:
            float: The state of the right trigger button ranging from 0.0 to 1.0.
        """
        return self._get_button_method("getRightTriggerAxis") + self.trigger_fix

    def lefttrigger(self) -> float:
        """
        Returns the state of the left trigger button.

        Returns:
            float: The state of the left trigger button ranging from 0.0 to 1.0.
        """
        return self._get_button_method("getLeftTriggerAxis") + self.trigger_fix

    def __pov_xy(self):
        """
        Returns the X and Y values of the POV as a tuple.

        Returns:
            tuple: The X and Y values of the POV as a tuple.
        """
        pov_value = self.pov()
        pov_mapping = {
            0: (1, 0),
            45: (0.707, -0.707),
            90: (0, -1),
            135: (-0.707, -0.707),
            180: (-1, 0),
            225: (-0.707, 0.707),
            270: (0, 1),
            315: (0.707, 0.707),
        }
        return pov_mapping.get(
            pov_value, (0, 0)
        )  # Return (0, 0) for unmapped POV values

    def pov_x(self) -> float:
        """
        Returns the X-axis value of the POV (Point of View) of a joystick.

        Example:
        ```
        controller = SmartController(0)

        if controller.pov() >= 0:
            left_joy_x = controller.pov_x()
            left_joy_y = controller.pov_y()
        ```

        Returns:
            float: The X-axis value of the POV.
        """
        return self.__pov_xy()[0]

    def pov_y(self) -> float:
        """
        Returns the Y-axis value of the POV (Point of View) of a joystick.

        Example:
        ```
        controller = SmartController(0)

        if controller.pov() >= 0:
            left_joy_x = controller.pov_x()
            left_joy_y = controller.pov_y()
        ```

        Returns:
            float: The Y-axis value of the POV.
        """
        return self.__pov_xy()[1]
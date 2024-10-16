import math

import wpilib
import wpilib.shuffleboard
from wpilib import (
    SmartDashboard,
    RobotController,
    XboxController,
    PS5Controller,
    DriverStation,
)
from wpimath import applyDeadband
from phoenix6.hardware import TalonFX, CANcoder


from util.smart_preference import SmartPreference, SmartProfile
from util.wrappers import SmartController

import magicbot
import navx


from components.sysid_drive import SysIdDrive
from components.swerve_drive import SwerveDrive
from components.swerve_wheel import SwerveWheel
from util.smart_preference import SmartPreference, SmartProfile
from container import RobotContainer

class MyRobot(magicbot.MagicRobot):
    sysid_drive: SysIdDrive

    swerve_drive: SwerveDrive
    front_left: SwerveWheel
    front_right: SwerveWheel
    rear_left: SwerveWheel
    rear_right: SwerveWheel

    """This should be the max speed (m/s) at which the drive motors can
    run, NOT the max speed that the robot should go (ie. use a curve
    instead). This is because this is also used to calculate omega."""
    max_speed = SmartPreference(3.0)

    def createObjects(self):

        self.contanier = RobotContainer()
        self.debug = True

        # Swerve Motor IDs
        self.front_left_speed_motor = TalonFX(11)
        self.front_left_direction_motor = TalonFX(12)
        self.front_left_cancoder = CANcoder(13)

        self.front_right_speed_motor = TalonFX(21)
        self.front_right_direction_motor = TalonFX(22)
        self.front_right_cancoder = CANcoder(23)

        self.rear_left_speed_motor = TalonFX(31)
        self.rear_left_direction_motor = TalonFX(32)
        self.rear_left_cancoder = CANcoder(33)

        self.rear_right_speed_motor = TalonFX(41)
        self.rear_right_direction_motor = TalonFX(42)
        self.rear_right_cancoder = CANcoder(43)

        # Swerve Drive
        self.navX = navx.AHRS.create_spi()
        self.offset_x = 0.381
        self.offset_y = 0.381
        self.drive_gear_ratio = 6.75
        self.wheel_radius = 0.0508

        self.speed_profile = SmartProfile("speed")
        self.direction_profile = SmartProfile(
            "direction", continuous_range=(0, math.tau)
        )
        SmartDashboard.putData("Speed Profile", self.speed_profile)
        SmartDashboard.putData("Direction Profile", self.direction_profile)
        self.navX.setAngleAdjustment(0)

        self.previous_angle = self.navX.getAngle()

    def teleopInit(self):
        self.navX.reset()
        self.navX.setAngleAdjustment(-90)

    def teleopPeriodic(self):

        port_number = 0
        smart_controller = SmartController(port_number)

        # Get the current POV from the controller
        pov_value = smart_controller.pov()  # Call the method on the instance
  


        mult = 1
        # Call bumper methods on the instance
        if smart_controller.leftbumper():  # Corrected to use the instance
            mult *= 0.5
        if smart_controller.rightbumper():  # Corrected to use the instance
            mult *= 0.5

        # Get joystick values
        left_joy_x = (
            applyDeadband(smart_controller.lefty(), 0.1) * mult * self.max_speed
        )
        left_joy_y = (
            applyDeadband(smart_controller.leftx(), 0.1) * mult * self.max_speed
        )

        # Define the POV-to-(left_joy_x, left_joy_y) mapping
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

        # Update the joystick values based on the POV value if it's in the mapping
        if pov_value in pov_mapping:
            left_joy_x, left_joy_y = pov_mapping[pov_value]
            left_joy_x *= mult * self.max_speed * -1
            left_joy_y *= mult * self.max_speed * -1

        # calculate max angular speed based on max_speed (cool math here)
        omega = self.max_speed / math.dist((0, 0), (self.offset_x, self.offset_y))
        right_joy_x = applyDeadband(smart_controller.rightx(), 0.1) * mult * omega

        if left_joy_x != 0 or left_joy_y != 0 or right_joy_x != 0:
            self.swerve_drive.drive(
                -left_joy_y, left_joy_x, -right_joy_x, self.max_speed, self.period
            )

        if smart_controller.startbutton():  # Corrected to use the instance
            self.swerve_drive.reset_gyro()
            self.navX.setAngleAdjustment(-90)

        if smart_controller.abutton():  # Corrected to use the instance
            self.sysid_drive.quasistatic_forward()
        if smart_controller.bbutton():  # Corrected to use the instance
            self.sysid_drive.quasistatic_reverse()
        if smart_controller.xbutton():  # Corrected to use the instance
            self.sysid_drive.dynamic_forward()
        if smart_controller.ybutton():  # Corrected to use the instance
            self.sysid_drive.dynamic_reverse()

        SmartDashboard.putNumber("Gyro Angle", self.navX.getAngle())
        SmartDashboard.putNumber("Voltage", RobotController.getBatteryVoltage())

        if right_joy_x == 0:
            # If there's no rotational input, calculate the gyro drift
            current_angle = self.navX.getAngle()
            gyro_drift = current_angle - self.previous_angle
            SmartDashboard.putNumber("Gyro Drift", gyro_drift)
        else:
            self.previous_angle = self.navX.getAngle()

    # override _do_periodics() to access watchdog
    # DON'T DO ANYTHING ELSE HERE UNLESS YOU KNOW WHAT YOU'RE DOING
    def _do_periodics(self):
        super()._do_periodics()
        self.period = max(0.02, self.watchdog.getTime())




if __name__ == "__main__":
    wpilib.run(MyRobot)

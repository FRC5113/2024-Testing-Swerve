import math

from pyfrc.physics.core import PhysicsInterface
from phoenix6 import unmanaged
from wpilib import DriverStation, RobotController
from wpilib.simulation import DCMotorSim
from wpimath.system.plant import DCMotor

from robot import MyRobot


class PhysicsEngine:
    def __init__(self, physics_controller: PhysicsInterface, robot: MyRobot):
        self.physics_controller = physics_controller
        self.robot = robot
        self.falcon_sim = DCMotorSim(DCMotor.falcon500(1), 6.75, 0.01)  # update gearing
        self.fl_speed = robot.front_left_speed_motor.sim_state
        self.fl_direction = robot.front_left_direction_motor.sim_state
        self.fl_encoder = robot.front_left_cancoder.sim_state
        self.fl_direction.add_rotor_position(-0.25)

    def update_sim(self, now, tm_diff):
        chassis_speeds = self.robot.swerve_drive.chassis_speeds
        pose = self.physics_controller.drive(chassis_speeds, tm_diff)
        self.robot.navX.setAngleAdjustment(-pose.rotation().degrees())

        if DriverStation.isEnabled():
            unmanaged.feed_enable(100)
        if not self.robot.swerve_drive.stopped:
            battery_v = RobotController.getBatteryVoltage()
            self.fl_speed.set_supply_voltage(battery_v)
            self.fl_direction.set_supply_voltage(battery_v)
            self.falcon_sim.setInputVoltage(self.fl_direction.motor_voltage)
            self.falcon_sim.update(tm_diff)
            # print(self.fl_direction.motor_voltage, self.falcon_sim.getAngularVelocity() / (2 * math.pi) / (150 / 7))
            self.fl_encoder.add_position(
                self.falcon_sim.getAngularVelocity() / (2 * math.pi) / (150 / 7)
            )  # convert
            # if this works ima kms
            self.fl_direction.add_rotor_position(
                self.falcon_sim.getAngularVelocity() / (2 * math.pi) / (150 / 7)
            )

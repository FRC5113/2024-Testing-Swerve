import math

from pyfrc.physics.core import PhysicsInterface
from pyfrc.physics.drivetrains import four_motor_swerve_drivetrain
from phoenix6 import unmanaged
from wpilib import DriverStation, RobotController
from wpilib.simulation import DCMotorSim
from wpimath.system.plant import DCMotor

from robot import MyRobot


class PhysicsEngine:
    def __init__(self, physics_controller: PhysicsInterface, robot: MyRobot):
        self.physics_controller = physics_controller
        self.robot = robot
        self.speed_sim_states = (
            robot.front_left_speed_motor.sim_state,
            robot.front_right_speed_motor.sim_state,
            robot.rear_left_speed_motor.sim_state,
            robot.rear_right_speed_motor.sim_state,
        )
        self.speed_falcon_sims = (
            DCMotorSim(DCMotor.falcon500(1), 6.75, 0.01),
            DCMotorSim(DCMotor.falcon500(1), 6.75, 0.01),
            DCMotorSim(DCMotor.falcon500(1), 6.75, 0.01),
            DCMotorSim(DCMotor.falcon500(1), 6.75, 0.01),
        )
        self.direction_sim_states = (
            robot.front_left_direction_motor.sim_state,
            robot.front_right_direction_motor.sim_state,
            robot.rear_left_direction_motor.sim_state,
            robot.rear_right_direction_motor.sim_state,
        )
        self.direction_falcon_sims = (
            DCMotorSim(DCMotor.falcon500(1), 150 / 7, 0.01),
            DCMotorSim(DCMotor.falcon500(1), 150 / 7, 0.01),
            DCMotorSim(DCMotor.falcon500(1), 150 / 7, 0.01),
            DCMotorSim(DCMotor.falcon500(1), 150 / 7, 0.01),
        )
        self.encoders = (
            robot.front_left_cancoder,
            robot.front_right_cancoder,
            robot.rear_left_cancoder,
            robot.rear_right_cancoder,
        )

    def update_sim(self, now, tm_diff):
        if DriverStation.isEnabled():
            unmanaged.feed_enable(100)
        if not self.robot.swerve_drive.stopped:
            battery_v = RobotController.getBatteryVoltage()
            for i in range(4):
                self.speed_sim_states[i].set_supply_voltage(battery_v)
                self.speed_falcon_sims[i].setInputVoltage(
                    self.speed_sim_states[i].motor_voltage
                )
                self.speed_falcon_sims[i].update(tm_diff)
                self.speed_sim_states[i].set_rotor_velocity(
                    self.speed_falcon_sims[i].getAngularVelocity()
                )
                self.direction_sim_states[i].set_supply_voltage(battery_v)
                self.direction_falcon_sims[i].setInputVoltage(
                    self.direction_sim_states[i].motor_voltage
                )
                self.direction_falcon_sims[i].update(tm_diff)
                self.encoders[i].sim_state.add_position(
                    self.direction_falcon_sims[i].getAngularVelocity()
                    / (2 * math.pi)
                    * tm_diff
                )

            chassis_speeds = self.robot.swerve_drive.chassis_speeds

            sim_speeds = four_motor_swerve_drivetrain(
                self.speed_sim_states[2].motor_voltage / battery_v,
                self.speed_sim_states[3].motor_voltage / battery_v,
                self.speed_sim_states[0].motor_voltage / battery_v,
                self.speed_sim_states[1].motor_voltage / battery_v,
                (self.encoders[2].get_absolute_position().value * -360) % 360,
                (self.encoders[3].get_absolute_position().value * -360) % 360,
                (self.encoders[0].get_absolute_position().value * -360) % 360,
                (self.encoders[1].get_absolute_position().value * -360) % 360,
                2.5,
                2.5,
                9.84,
            )
            # artificially soften simulated omega
            sim_speeds.omega_dps *= 0.4
            pose = self.physics_controller.drive(sim_speeds, tm_diff)
            self.robot.navX.setAngleAdjustment(-pose.rotation().degrees())

import wpilib.simulation
from pyfrc.physics.core import PhysicsInterface
from pyfrc.physics import motor_cfgs
from pyfrc.physics.drivetrains import four_motor_swerve_drivetrain
from pyfrc.physics.units import units

class PhysicsEngine:
    def __init__(self, physics_controller: PhysicsInterface, robot: "MyRobot"):
        self.physics_controller = physics_controller
        self.robot = robot

    def update_sim(self, now, tm_diff):
        chassis_speeds = self.robot.swerve_drive.chassis_speeds
        pose = self.physics_controller.drive(chassis_speeds, tm_diff)
        self.robot.navX.setAngleAdjustment(-pose.rotation().degrees())
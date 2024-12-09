from phoenix6.hardware import Pigeon2
from wpimath.geometry import Rotation2d, Rotation3d


class LemonPigeon(Pigeon2):
    """
    Wrapper for the Pigeon2 that makes it easier to use and some fetures in
    beta and back ports them.
    Acts like a normal Pigeon2 object but with some extra features.
    """

    def __init__(self, device_id, can_bus="rio"):
        """
        Initialize the Pigeon Wrapper.

        :param device_id: The CAN ID of the Pigeon device.
        :param can_bus: The CAN bus name (default is "rio").
        """
        self.pigeon = Pigeon2(device_id, can_bus)

    def get_yaw(self):
        """
        Get the yaw angle of the robot.

        :return: The yaw in degrees.
        """
        return self.pigeon.get_yaw()

    def set_yaw(self, angle):
        """
        Set the yaw angle of the robot.

        :param angle: The angle in degrees to set the yaw to.
        """
        self.pigeon.set_yaw(angle)

    def get_pitch(self):
        """
        Get the pitch angle of the robot.

        :return: The pitch in degrees.
        """
        return self.pigeon.get_pitch()

    def get_roll(self):
        """
        Get the roll angle of the robot.

        :return: The roll in degrees.
        """
        return self.pigeon.get_roll()

    def reset(self):
        """
        Reset the yaw angle to zero.
        """
        self.pigeon.set_yaw(0)

    def getRotation2d(self) -> Rotation2d:
        """
        Get the yaw angle as a Rotation2d object.

        :return: A Rotation2d object representing the yaw.
        """
        yaw_degrees = self.get_yaw().value
        return Rotation2d.fromDegrees(yaw_degrees)

    def get_rotation3d(self):
        """
        Get the yaw, pitch, and roll angles as a Rotation3d object.

        :return: A Rotation3d object representing the yaw, pitch, and roll.
        """
        yaw_degrees = self.get_yaw()
        pitch_degrees = self.get_pitch()
        roll_degrees = self.get_roll()
        return Rotation3d.fromDegrees(yaw_degrees, pitch_degrees, roll_degrees)

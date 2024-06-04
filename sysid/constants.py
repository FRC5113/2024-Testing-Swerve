#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import math


class DriveConstants:
    # The PWM IDs for the drivetrain motor controllers.
    kLeftMotor1Port = 11
    kLeftMotor2Port = 31
    kRightMotor1Port = 21
    kRightMotor2Port = 41

    # Encoders and their respective motor controllers.
    kLeftEncoderPorts = (0, 1)
    kRightEncoderPorts = (2, 3)
    kLeftEncoderReversed = False
    kRightEncoderReversed = False  # True ??????

    # Encoder counts per revolution/rotation.
    kEncoderCPR = 1.0
    kWheelDiameterInches = 4.0

    # Assumes the encoders are directly mounted on the wheel shafts
    kEncoderDistancePerPulse = (kWheelDiameterInches * math.pi) / kEncoderCPR


# autonomous
class AutonomousConstants:
    kTimeoutSeconds = 3.0
    kDriveDistanceMetres = 2.0
    kDriveSpeed = 0.5


class OIConstants:
    kDriverControllerPort = 0

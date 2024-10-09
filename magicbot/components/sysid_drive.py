from commands2.sysid import SysIdRoutine

from util.magic_sysid_routine import MagicSysIdRoutine
from components.swerve_drive import SwerveDrive


class SysIdDrive(MagicSysIdRoutine):
    swerve_drive: SwerveDrive

    def setup(self):
        self.setup_sysid(
            SysIdRoutine.Config(rampRate=0.2, stepVoltage=7.0),
            SysIdRoutine.Mechanism(
                self.swerve_drive.sysid_drive,
                self.swerve_drive.sysid_log,
                self.swerve_drive,
                "Drive",
            ),
        )

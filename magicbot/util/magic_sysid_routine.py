from wpilib import Timer
from commands2.sysid import SysIdRoutine
from wpilib.sysid import SysIdRoutineLog, State
from magicbot import will_reset_to


class MagicSysIdRoutine:
    enabled = will_reset_to(False)
    output_volts = will_reset_to(0)

    def __init__(self):
        self.timer = Timer()
        self.timed_out = False
        self.was_enabled = False
        self.state = State.kNone

    def setup_sysid(
        self, config: SysIdRoutine.Config, mechanism: SysIdRoutine.Mechanism
    ):
        self.log = SysIdRoutineLog(mechanism.name)
        self.config = config
        self.mechanism = mechanism
        self.record_state = config.recordState or self.log.recordState

    def quasistatic_forward(self):
        self.enabled = True
        self.state = State.kQuasistaticForward
        self.outputVolts = self.timer.get() * self.config.rampRate

    def quasistatic_reverse(self):
        self.enabled = True
        self.state = State.kQuasistaticReverse
        self.outputVolts = -self.timer.get() * self.config.rampRate

    def dynamic_forward(self):
        self.enabled = True
        self.state = State.kDynamicForward
        self.outputVolts = self.config.stepVoltage

    def dynamic_reverse(self):
        self.enabled = True
        self.state = State.kDynamicReverse
        self.outputVolts = -self.config.stepVoltage

    def on_start(self):
        self.timer.restart()
        self.timed_out = False
        self.was_enabled = True

    def on_end(self):
        self.was_enabled = False
        self.mechanism.drive(0.0)
        self.record_state(State.kNone)
        self.timer.stop()

    def execute(self):
        if self.was_enabled:
            if self.timed_out:
                return
            if not self.enabled:
                self.on_end()
                return
        else:
            if not self.enabled:
                return
            self.on_start()

        if self.timer.get() > self.config.timeout:
            self.on_end()
            self.timed_out = True
            return

        self.mechanism.drive(self.outputVolts)
        self.mechanism.log(self.log)
        self.record_state(self.state)

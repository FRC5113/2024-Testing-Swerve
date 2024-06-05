import wpilib
from wpilib import Preferences


class Controler:
    
    
    def xbox(self):
        self.getAbutton = wpilib.XboxController.getAButton
        self.getBbutton = wpilib.XboxController.getBButton
        self.getLeftX = wpilib.XboxController.getLeftX
        self.getLeftY = wpilib.XboxController.getLeftY
        self.getRightX= wpilib.XboxController.getRightX
        self.getRightY= wpilib.XboxController.getRightY


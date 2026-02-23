from wpilib import Joystick, RobotBase

from adaptive_robot.adaptive_robot import AdaptiveRobot

from controls.drivetrain_control import DrivetrainControl

from subsystems.drivetrain.drivetrain import Drivetrain
from subsystems.drivetrain.drivetrain_io import SimDrivetrainIO, RealDrivetrainIO

from utils.priorities import DrivetrainPriority


class MyRobot(AdaptiveRobot):
    def onRobotInit(self) -> None:
        self.controller = Joystick(0)

        if RobotBase.isSimulation():
            self.drive_io = SimDrivetrainIO()
        else:
            self.drive_io = RealDrivetrainIO()

        self.drivetrain = Drivetrain(self, self.drive_io)
        self.drivetrain_control = DrivetrainControl(self, self.drivetrain, self.controller)

    def disabledPeriodic(self) -> None:
        self.drivetrain.request_drivetrain_stop(DrivetrainPriority.SAFETY, "safety")

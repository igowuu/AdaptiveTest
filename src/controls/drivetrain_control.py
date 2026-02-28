from wpilib import DriverStation, Joystick

from adaptive_robot.adaptive_robot import AdaptiveRobot
from adaptive_robot.adaptive_component import AdaptiveComponent

from subsystems.drivetrain.drivetrain import Drivetrain

from utils.priorities import DrivetrainPriority


class DrivetrainControl(AdaptiveComponent):
    def __init__(
        self, 
        robot: AdaptiveRobot, 
        drivetrain: Drivetrain, 
        controller: Joystick
    ) -> None:
        super().__init__(robot, "DrivetrainControl")

        self.drivetrain = drivetrain

        self.controller = controller

        self.drive_percent = self.tunable("Tunables/DrivePCT", 0.8)

    def execute(self) -> None:
        """
        Declare when the drivetrain should move based on user input.
        This method is automatically called each iteration by the scheduler.
        """
        if not DriverStation.isTeleopEnabled():
            return
        
        forward_speed_pct = -self.controller.getY() * self.drive_percent.value
        angular_speed_pct = -self.controller.getX() * self.drive_percent.value
        
        self.drivetrain.request_linear_percent(forward_speed_pct, DrivetrainPriority.TELEOP, "teleop")
        self.drivetrain.request_angular_percent(angular_speed_pct, DrivetrainPriority.TELEOP, "teleop")

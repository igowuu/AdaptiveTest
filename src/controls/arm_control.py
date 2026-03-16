from wpilib import DriverStation, Joystick

from adaptive_robot.adaptive_robot import AdaptiveRobot
from adaptive_robot.adaptive_component import AdaptiveComponent

from subsystems.arm.arm import Arm

from utils.priorities import ArmPriority


class ArmControl(AdaptiveComponent):
    def __init__(
        self, 
        robot: AdaptiveRobot, 
        arm: Arm, 
        controller: Joystick
    ) -> None:
        super().__init__(robot, "ArmControl")

        self.arm = arm

        self.controller = controller

    def execute(self) -> None:
        """
        Declare when the arm should move based on user input.
        This method is automatically called each iteration by the scheduler.
        """
        if not DriverStation.isTeleopEnabled():
            return

        if self.controller.getRawButton(1):
            self.arm.request_arm_angle(3, ArmPriority.TELEOP, "teleop")
        elif self.controller.getRawButton(2):
            self.arm.request_arm_angle(0, ArmPriority.TELEOP, "teleop")
        else:
            self.arm.request_arm_stop(ArmPriority.SAFETY, "safety")

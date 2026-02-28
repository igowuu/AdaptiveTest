from adaptive_robot.adaptive_robot import AdaptiveRobot
from adaptive_robot.adaptive_component import AdaptiveComponent


class Intake(AdaptiveComponent):
    def __init__(self, robot: AdaptiveRobot) -> None:
        super().__init__(robot, "Intake")


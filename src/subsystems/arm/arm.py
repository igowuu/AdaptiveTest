from wpimath.controller import ArmFeedforward
from wpimath.units import radians

from adaptive_robot.adaptive_robot import AdaptiveRobot
from adaptive_robot.adaptive_component import AdaptiveComponent
from adaptive_robot.requests import AxisController

from subsystems.arm.arm_io import ArmIO

from utils.priorities import ArmPriority

from config.constants import ArmConst, ArmPositionPID, ArmFF


class Arm(AdaptiveComponent):
    def __init__(self, robot: "AdaptiveRobot", io: ArmIO) -> None:
        super().__init__(robot, "Arm")

        self.io = io

        self.arm_position_pid = self.tunablePID(
            ArmPositionPID.KP, 
            ArmPositionPID.KI, 
            ArmPositionPID.KD,
            directory="Tunables/Arm/ArmPositionPID"
        )

        self.arm_ff = ArmFeedforward(ArmFF.KS, ArmFF.KG, ArmFF.KV, ArmFF.KA)

        self.arm_position_controller = AxisController()

        self.previous_arm_position = 0.0

    def request_arm_stop(
        self,
        priority: ArmPriority,
        source: str = "unknown"
    ) -> None:
        """
        Requests the arm to hold its position.
        """
        self.arm_position_controller.request(self.io.get_angle(), priority.value, source)

    def request_arm_angle(
        self, 
        desired_angle: radians,
        priority: ArmPriority,
        source: str = "unknown"
    ) -> None:
        """
        Requests an angle in radians to be applied to the arm.
        """
        self.arm_position_controller.request(desired_angle, priority.value, source)

    def _move_to_angle(self, target_angle: radians) -> None:
        """
        Moves the arm to a specific angle in radians using position control with PID and feedforward.
        """
        current_angle = self.io.get_angle()
        current_velocity = self.io.get_velocity()

        pid_volts = self.arm_position_pid.calculate(current_angle, target_angle)
        ff_volts = self.arm_ff.calculate(current_angle, current_velocity)

        self.io.command_voltage(pid_volts + ff_volts)

    def publish_telemetry(self) -> None:
        """
        Publishes all arm-specific data before execute() is run.
        """
        resolved_position = self.arm_position_controller.resolve()
        arm_angle = self.io.get_angle()

        self.publish_value("Arm/Angle (radians)", arm_angle)

        self.publish_value("Arm/MinLimitReached", arm_angle <= ArmConst.MIN_ANGLE)
        self.publish_value("Arm/MaxLimitReached", arm_angle >= ArmConst.MAX_ANGLE)

        self.publish_value("Arm/ArmMotorVoltage", self.io.get_voltage())

        self.publish_value("Arm/ArmAxis/ResolvedPosition (radians)", resolved_position.value)
        self.publish_value("Arm/ArmAxis/Mode", resolved_position.source)
        self.publish_value("Arm/ArmAxis/Priority", resolved_position.priority)

    def on_faulted(self) -> None:
        """
        Method called automatically by the scheduler each iteration if arm is unhealthy.
        """
        self.request_arm_stop(ArmPriority.SAFETY, "safety")

    def execute(self) -> None:
        """
        Method automatically called at the end of each iteration by the scheduler.
        """
        requested_position = self.arm_position_controller.resolve()

        self._move_to_angle(requested_position.value)

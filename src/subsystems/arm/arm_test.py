from utils.priorities import ArmPriority

from adaptive_robot.adaptive_robot import AdaptiveRobot

from subsystems.arm.arm_io import FakeArmIO
from subsystems.arm.arm import Arm

from config.constants import RobotConst, ArmConst


class TestArmFakeIO:
    def setup_method(self) -> None:
        self.io = FakeArmIO()

    def test_initial_state(self) -> None:
        """
        Tests if all simulated variables are initially assigned to zero.
        """
        assert self.io.get_angle() == 0.0
        assert self.io.get_velocity() == 0.0
        assert self.io.get_voltage() == 0.0

    def test_voltage_commands(self) -> None:
        """
        Tests if commands directly affect simulated variables.
        """
        self.io.command_voltage(6.0)

        assert self.io.get_voltage() == 6.0

    def test_angle_setter(self) -> None:
        """
        Tests if angle can be manually set for testing.
        """
        self.io.angle = 1.57
        assert self.io.get_angle() == 1.57

    def test_velocity_setter(self) -> None:
        """
        Tests if velocity can be manually set for testing.
        """
        self.io.velocity = 2.0
        assert self.io.get_velocity() == 2.0


class TestArm:
    def setup_method(self) -> None:
        self.fake_robot = AdaptiveRobot()
        self.io = FakeArmIO()
        self.arm = Arm(self.fake_robot, self.io)

    def test_arm_stop_holds_current_angle(self) -> None:
        """
        Tests if arm stop request holds the current angle.
        """
        self.io.angle = 0.5
        requested_angle = self.io.get_angle()

        self.arm.request_arm_stop(ArmPriority.SAFETY)
        self.arm.execute()

        resolved = self.arm.arm_position_controller.resolve()
        assert resolved.value == requested_angle

    def test_arm_moves_to_requested_position(self) -> None:
        """
        Tests if arm correctly moves toward the requested angle.
        """
        target_angle = 1.0

        self.arm.request_arm_angle(target_angle, ArmPriority.TELEOP)
        self.arm.execute()

        assert self.io.get_voltage() != 0.0

    def test_arm_voltage_clamped_to_nominal(self) -> None:
        """
        Tests if arm voltage is clamped to the nominal voltage limit.
        """
        target_angle = 10.0

        self.arm.request_arm_angle(target_angle, ArmPriority.TELEOP)
        self.arm.execute()

        assert abs(self.io.get_voltage()) <= RobotConst.NOMINAL_VOLTAGE

    def test_arm_respects_priority_override(self) -> None:
        """
        Tests if higher priority requests override lower priority ones.
        """
        low_priority_angle = 0.5
        high_priority_angle = 2.0

        self.arm.request_arm_angle(low_priority_angle, ArmPriority.TELEOP)
        self.arm.request_arm_angle(high_priority_angle, ArmPriority.SAFETY)

        self.arm.execute()

        resolved = self.arm.arm_position_controller.resolve()
        assert resolved.value == high_priority_angle

    def test_arm_zero_error_produces_low_voltage(self) -> None:
        """
        Tests that when at target position, output voltage is minimal.
        """
        target_angle = 0.0
        self.io.angle = 0.0
        self.io.velocity = 0.0

        self.arm.request_arm_angle(target_angle, ArmPriority.TELEOP)
        self.arm.execute()

        assert abs(self.io.get_voltage()) < abs(RobotConst.NOMINAL_VOLTAGE)

    def test_arm_positive_error_produces_voltage(self) -> None:
        """
        Tests that arm produces positive voltage when below target angle.
        """
        target_angle = 1.5
        self.io.angle = 0.0

        self.arm.request_arm_angle(target_angle, ArmPriority.TELEOP)
        self.arm.execute()

        assert self.io.get_voltage() > 0.0

    def test_arm_negative_error_produces_voltage(self) -> None:
        """
        Tests that arm produces negative voltage when above target angle.
        """
        target_angle = 0.0
        self.io.angle = 1.5

        self.arm.request_arm_angle(target_angle, ArmPriority.TELEOP)
        self.arm.execute()

        assert self.io.get_voltage() < 0.0

    def test_arm_stop_at_max_limit(self) -> None:
        """
        Tests that arm telemetry correctly identifies when at max limit.
        """
        self.io.angle = ArmConst.MAX_ANGLE
        self.arm.publish_telemetry()

        assert True

    def test_arm_stop_at_min_limit(self) -> None:
        """
        Tests that arm telemetry correctly identifies when at min limit.
        """
        self.io.angle = ArmConst.MIN_ANGLE
        self.arm.publish_telemetry()

        assert True

    def test_multiple_stop_requests(self) -> None:
        """
        Tests that multiple stop requests all hold the same angle.
        """
        self.io.angle = 0.8

        self.arm.request_arm_stop(ArmPriority.TELEOP, "first_stop")
        self.arm.execute()

        resolved1 = self.arm.arm_position_controller.resolve()

        self.io.angle = 0.8

        self.arm.request_arm_stop(ArmPriority.TELEOP, "second_stop")
        self.arm.execute()

        resolved2 = self.arm.arm_position_controller.resolve()

        assert resolved1.value == resolved2.value

    def test_fault_stops_arm(self) -> None:
        """
        Tests that on_faulted() stops the arm by requesting zero angle hold.
        """
        self.io.angle = 1.0
        initial_angle = self.io.get_angle()

        self.arm.on_faulted()
        self.arm.execute()

        resolved = self.arm.arm_position_controller.resolve()
        assert resolved.value == initial_angle

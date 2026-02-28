import math

from wpimath.geometry import Rotation2d

from utils.priorities import DrivetrainPriority

from adaptive_robot.adaptive_robot import AdaptiveRobot

from subsystems.drivetrain.drivetrain_io import FakeDrivetrainIO
from subsystems.drivetrain.drivetrain import Drivetrain, DriveMode

from config.constants import RobotConst


class TestDrivetrainSimIO:
    def setup_method(self) -> None:
        self.io = FakeDrivetrainIO()

    def test_initial_state(self) -> None:
        """
        Tests if all simulated variables are initially assigned to zero.
        """
        assert self.io.get_left_voltage() == 0.0
        assert self.io.get_right_voltage() == 0.0
        assert self.io.get_left_distance() == 0.0
        assert self.io.get_right_distance() == 0.0
        assert self.io.get_left_velocity() == 0.0
        assert self.io.get_right_velocity() == 0.0
        assert self.io.get_heading().radians() == 0.0

    def test_voltage_commands(self) -> None:
        """
        Tests if commands directly affect simulated variables.
        """
        self.io.command_left_voltage(6.0)
        self.io.command_right_voltage(-3.0)

        assert self.io.get_left_voltage() == 6.0
        assert self.io.get_right_voltage() == -3.0

    def test_position_setters(self) -> None:
        """
        Tests if position setters directly affect simulated variables.
        """
        self.io.set_left_position(2.5)
        self.io.set_right_position(4.0)

        assert self.io.get_left_distance() == 2.5
        assert self.io.get_right_distance() == 4.0

    def test_reset_gyro(self) -> None:
        """
        Tests if resetting the gyro resets yaw to zero.
        """
        self.io.heading = Rotation2d.fromDegrees(90)
        assert self.io.get_heading().degrees() == 90

        self.io.reset_gyro()

        assert self.io.get_heading().degrees() == 0


class TestDrivetrain:
    def setup_method(self) -> None:
        self.fake_robot = AdaptiveRobot()
        self.io = FakeDrivetrainIO()
        self.drivetrain = Drivetrain(self.fake_robot, self.io)

    def test_open_loop_applies_voltage(self) -> None:
        """
        Tests if drivetrains open loop method correctly applies the given voltage.
        """
        self.drivetrain.set_drive_mode(DriveMode.OPEN_LOOP)

        self.drivetrain.request_linear_percent(1.0, priority=DrivetrainPriority.TEST)
        self.drivetrain.execute()

        assert self.io.get_left_voltage() != 0
        assert self.io.get_right_voltage() != 0

    def test_in_place_turn_opposite_voltages(self) -> None:
        """
        Tests if a pure rotation command drives left/right in opposite directions.
        """
        self.drivetrain.set_drive_mode(DriveMode.OPEN_LOOP)

        self.drivetrain.request_angular_percent(1.0, priority=DrivetrainPriority.TEST)
        self.drivetrain.execute()

        assert self.io.get_left_voltage() * self.io.get_right_voltage() < 0.0

    def test_closed_loop_zero_request_holds_zero(self) -> None:
        """
        Tests that closed-loop with zero request outputs zero voltage.
        """
        self.drivetrain.set_drive_mode(DriveMode.CLOSED_LOOP)

        self.io.left_velocity = 0.0
        self.io.right_velocity = 0.0

        self.drivetrain.request_drivetrain_stop(priority=DrivetrainPriority.TEST)
        self.drivetrain.execute()

        assert self.io.get_left_voltage() == 0.0
        assert self.io.get_right_voltage() == 0.0

    def test_voltage_clamps_to_nominal(self) -> None:
        """
        Tests if voltage clamping correctly limits absurd values.
        """
        self.drivetrain.request_linear_percent(10.0, priority=DrivetrainPriority.TEST)
        self.drivetrain.execute()

        assert abs(self.io.get_left_voltage()) <= RobotConst.NOMINAL_VOLTAGE

    def test_closed_loop_clamps_to_nominal(self) -> None:
        """
        Tests if closed-loop voltage clamping limits absurd requests.
        """
        self.drivetrain.set_drive_mode(DriveMode.CLOSED_LOOP)

        self.io.left_velocity = 0.0
        self.io.right_velocity = 0.0

        self.drivetrain.request_linear_percent(10.0, priority=DrivetrainPriority.TEST)
        self.drivetrain.execute()

        assert abs(self.io.get_left_voltage()) <= RobotConst.NOMINAL_VOLTAGE
        assert abs(self.io.get_right_voltage()) <= RobotConst.NOMINAL_VOLTAGE

    def test_priority_override(self) -> None:
        """
        Tests if requests with higher priorities override ones with lower priorities.
        """
        self.drivetrain.request_linear_percent(0.5, DrivetrainPriority.TELEOP)
        self.drivetrain.request_linear_percent(1.0, DrivetrainPriority.SAFETY)

        self.drivetrain.execute()

        expected_voltage = RobotConst.NOMINAL_VOLTAGE
        assert abs(self.io.get_left_voltage()) <= expected_voltage
        assert self.io.get_left_voltage() > 0

    def test_odometry_updates_from_sensors(self) -> None:
        """
        Tests if odometry updates using the IO's sensor readings.
        """
        self.drivetrain.reset_odometry()

        self.io.set_left_position(1.0)
        self.io.set_right_position(1.0)
        self.io.heading = Rotation2d()

        self.drivetrain.request_drivetrain_stop(priority=DrivetrainPriority.TEST)
        self.drivetrain.execute()

        pose = self.drivetrain.get_pose()
        assert math.isclose(pose.X(), 1.0, abs_tol=1e-6)
        assert math.isclose(pose.Y(), 0.0, abs_tol=1e-6)
        assert math.isclose(pose.rotation().radians(), 0.0, abs_tol=1e-6)

    def test_on_faulted_zeroes_outputs(self) -> None:
        """
        Tests if fault handling forces outputs to zero.
        """
        self.drivetrain.set_drive_mode(DriveMode.OPEN_LOOP)
        self.drivetrain.request_linear_percent(1.0, priority=DrivetrainPriority.TEST)
        self.drivetrain.execute()

        assert self.io.get_left_voltage() != 0.0
        assert self.io.get_right_voltage() != 0.0

        self.drivetrain.on_faulted()

        assert self.io.get_left_voltage() == 0.0
        assert self.io.get_right_voltage() == 0.0

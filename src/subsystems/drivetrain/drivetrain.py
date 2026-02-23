from enum import Enum, auto

from wpilib import Field2d, SmartDashboard

from wpimath.units import percent, meters_per_second
from wpimath.geometry import Pose2d
from wpimath.kinematics import DifferentialDriveKinematics, DifferentialDriveOdometry, ChassisSpeeds
from wpimath.controller import SimpleMotorFeedforwardMeters
from wpimath.filter import SlewRateLimiter

from adaptive_robot.adaptive_robot import AdaptiveRobot
from adaptive_robot.adaptive_component import AdaptiveComponent
from adaptive_robot.requests import AxisController

from subsystems.drivetrain.drivetrain_io import DrivetrainIO

from utils.priorities import DrivetrainPriority
from utils.math_utils import clamp

from config.constants import RobotConst, DrivetrainConst, DrivetrainFF, DrivetrainPID


class DriveMode(Enum):
    OPEN_LOOP = auto()
    CLOSED_LOOP = auto()


class Drivetrain(AdaptiveComponent):
    def __init__(self, robot: AdaptiveRobot, io: DrivetrainIO) -> None:
        super().__init__(robot, "Drivetrain")

        self.io = io

        self.kinematics = DifferentialDriveKinematics(DrivetrainConst.TRACK_WIDTH)

        self.left_ff = SimpleMotorFeedforwardMeters(
            kS=DrivetrainFF.LEFT_KS,
            kV=DrivetrainFF.LEFT_KV,
            kA=DrivetrainFF.LEFT_KA
        )
        self.right_ff = SimpleMotorFeedforwardMeters(
            kS=DrivetrainFF.RIGHT_KS,
            kV=DrivetrainFF.RIGHT_KV,
            kA=DrivetrainFF.RIGHT_KA
        )

        self.left_pid = self.tunablePID(
            kp=DrivetrainPID.LEFT_KP,
            ki=DrivetrainPID.LEFT_KI,
            kd=DrivetrainPID.LEFT_KD,
            directory="Tunables/Drivetrain/LeftPID"
        )
        self.right_pid = self.tunablePID(
            kp=DrivetrainPID.RIGHT_KP,
            ki=DrivetrainPID.RIGHT_KI,
            kd=DrivetrainPID.RIGHT_KD,
            directory="Tunables/Drivetrain/RightPID"
        )

        self.odometry = DifferentialDriveOdometry(
            gyroAngle=self.io.get_heading(),
            leftDistance=self.io.get_left_distance(),
            rightDistance=self.io.get_right_distance()
        )

        self.left_voltage_slew = SlewRateLimiter(DrivetrainConst.SLEW_LIMIT)
        self.right_voltage_slew = SlewRateLimiter(DrivetrainConst.SLEW_LIMIT)

        self.linear_percent_controller = AxisController()
        self.angular_percent_controller = AxisController()

        self.drive_mode = DriveMode.CLOSED_LOOP

        self.field = Field2d()
        SmartDashboard.putData("Field", self.field)

        self.previous_left_velocity = 0.0
        self.previous_right_velocity = 0.0
    
    def get_pose(self) -> Pose2d:
        """
        Returns the drivetrain pose from odometry.
        """
        return self.odometry.getPose()

    def set_drive_mode(self, mode: DriveMode) -> None:
        """
        Sets the drivetrain mode between open-loop and closed-loop control.
        """
        self.drive_mode = mode

    def reset_odometry(self, pose: Pose2d = Pose2d()) -> None:
        """
        Zeroes odometry and all drivetrain all sensors.
        """
        self.io.reset_gyro()

        self.io.set_left_position(0.0)
        self.io.set_right_position(0.0)
        
        self.previous_left_velocity = 0.0
        self.previous_right_velocity = 0.0

        self.odometry.resetPosition(
            gyroAngle=self.io.get_heading(),
            leftDistance=0.0,
            rightDistance=0.0,
            pose=pose
        )

    def request_drivetrain_stop(
        self,
        priority: DrivetrainPriority,
        name: str = "unknown"
    ) -> None:
        """
        Requests all linear and angular drivetrain movement to terminate.
        """
        self.linear_percent_controller.request(0.0, priority.value, name)
        self.angular_percent_controller.request(0.0, priority.value, name)

    def request_linear_percent(
        self,
        percent: percent,
        priority: DrivetrainPriority,
        name: str = "unknown"
    ) -> None:
        """
        Requests a linear speed to be applied to the drivetrain.
        """
        self.linear_percent_controller.request(percent, priority.value, name)

    def request_angular_percent(
        self,
        percent: percent,
        priority: DrivetrainPriority,
        name: str = "unknown"
    ) -> None:
        """
        Requests an angular speed to be applied to the drivetrain.
        """
        self.angular_percent_controller.request(percent, priority.value, name)

    def _drive_open_loop(
        self,
        desired_left_velocity: meters_per_second,
        desired_right_velocity: meters_per_second
    ) -> None:
        """
        Drives the left and right motors given desired percents for the left and right motors.
        Does not apply FF + PID.
        """
        left_percent = desired_left_velocity / DrivetrainConst.MAX_SPEED_MPS
        right_percent = desired_right_velocity / DrivetrainConst.MAX_SPEED_MPS

        left_voltage = left_percent * RobotConst.NOMINAL_VOLTAGE
        right_voltage = right_percent * RobotConst.NOMINAL_VOLTAGE

        clamped_left_voltage = clamp(left_voltage, -RobotConst.NOMINAL_VOLTAGE, RobotConst.NOMINAL_VOLTAGE)
        clamped_right_voltage = clamp(right_voltage, -RobotConst.NOMINAL_VOLTAGE, RobotConst.NOMINAL_VOLTAGE)

        desired_left_volts = self.left_voltage_slew.calculate(clamped_left_voltage)
        desired_right_volts = self.right_voltage_slew.calculate(clamped_right_voltage)

        self.previous_left_velocity = desired_left_velocity
        self.previous_right_velocity = desired_right_velocity

        self.io.command_left_voltage(desired_left_volts)
        self.io.command_right_voltage(desired_right_volts)

    def _drive_closed_loop(
        self,
        desired_left_velocity: meters_per_second,
        desired_right_velocity: meters_per_second
    ) -> None:
        """
        Drives the left and right motors given desired velocities for the left and right motors.
        Applies FF + PID.
        """
        measured_left_velocity = self.io.get_left_velocity()
        measured_right_velocity = self.io.get_right_velocity()

        left_ff_volts = self.left_ff.calculate(self.previous_left_velocity, desired_left_velocity)
        right_ff_volts = self.right_ff.calculate(self.previous_right_velocity, desired_right_velocity)

        left_pid_volts = self.left_pid.calculate(measured_left_velocity, desired_left_velocity)
        right_pid_volts = self.right_pid.calculate(measured_right_velocity, desired_right_velocity)

        left_volts = left_ff_volts + left_pid_volts
        right_volts = right_ff_volts + right_pid_volts

        clamped_left_volts = clamp(left_volts, -RobotConst.NOMINAL_VOLTAGE, RobotConst.NOMINAL_VOLTAGE)
        clamped_right_volts = clamp(right_volts, -RobotConst.NOMINAL_VOLTAGE, RobotConst.NOMINAL_VOLTAGE)

        desired_left_volts = self.left_voltage_slew.calculate(clamped_left_volts)
        desired_right_volts = self.right_voltage_slew.calculate(clamped_right_volts)

        self.previous_left_velocity = desired_left_velocity
        self.previous_right_velocity = desired_right_velocity

        self.io.command_left_voltage(desired_left_volts)
        self.io.command_right_voltage(desired_right_volts)

    def publish_telemetry(self) -> None:
        """
        Method called automatically by the scheduler at the end of each iteration, before execute().
        """
        resolved_linear = self.linear_percent_controller.resolve()
        resolved_angular = self.angular_percent_controller.resolve()

        self.publish_value("Drive/Sensors/GyroAngle (radians)", self.io.get_heading().radians())

        self.publish_value("Drive/Sensors/LeftDistance (meters)", self.io.get_left_distance())
        self.publish_value("Drive/Sensors/RightDistance (meters)", self.io.get_right_distance())

        self.publish_value("Drive/Sensors/LeftVelocity (mps)", self.io.get_left_velocity())
        self.publish_value("Drive/Sensors/RightVelocity (mps)", self.io.get_right_velocity())
        
        self.publish_value("Drive/Sensors/LeftAppliedVoltage", self.io.get_left_voltage())
        self.publish_value("Drive/Sensors/RightAppliedVoltage", self.io.get_right_voltage())

        self.publish_value("Drive/Odometry/X (meters)", self.get_pose().X())
        self.publish_value("Drive/Odometry/Y (meters)", self.get_pose().Y())
        self.publish_value("Drive/Odometry/Yaw (radians)", self.get_pose().rotation().radians())

        self.publish_value("Drive/DriveAxis/ResolvedX (percent)", resolved_linear.value)
        self.publish_value("Drive/DriveAxis/ForwardMode", resolved_linear.source)
        self.publish_value("Drive/DriveAxis/ForwardPriority", resolved_linear.priority)

        self.publish_value("Drive/DriveAxis/ResolvedYaw (percent)", resolved_angular.value)
        self.publish_value("Drive/DriveAxis/RotationMode", resolved_angular.source)
        self.publish_value("Drive/DriveAxis/RotationPriority", resolved_angular.priority)

    def execute(self) -> None:
        """
        Method called automatically by the scheduler at the end of each iteration.
        """
        resolved_linear = self.linear_percent_controller.resolve().value
        resolved_angular = self.angular_percent_controller.resolve().value
        
        linear_velocity = resolved_linear * DrivetrainConst.MAX_SPEED_MPS
        angular_velocity = resolved_angular * DrivetrainConst.MAX_SPEED_RADPS

        desired_chassis = ChassisSpeeds(linear_velocity, 0.0, angular_velocity)

        wheel_speeds = self.kinematics.toWheelSpeeds(desired_chassis)
        wheel_speeds.desaturate(DrivetrainConst.MAX_SPEED_MPS)

        if self.drive_mode == DriveMode.CLOSED_LOOP:
            self._drive_closed_loop(wheel_speeds.left, wheel_speeds.right)
        else:
            self._drive_open_loop(wheel_speeds.left, wheel_speeds.right)

        self.io.update()
        self.odometry.update(
            gyroAngle=self.io.get_heading(), 
            leftDistance=self.io.get_left_distance(), 
            rightDistance=self.io.get_right_distance()
        )

        self.linear_percent_controller.clear()
        self.angular_percent_controller.clear()

        self.field.setRobotPose(self.get_pose())

from abc import ABC, abstractmethod

from navx import AHRS
from phoenix5 import TalonSRX, TalonSRXControlMode, NeutralMode

from wpilib.simulation import DifferentialDrivetrainSim

from wpimath.units import volts, meters, meters_per_second
from wpimath.geometry import Rotation2d, Pose2d
from wpimath.system.plant import DCMotor

from adaptive_robot.hardware.adaptive_dutycycle_encoder import AdaptiveDutyCycleEncoder
from adaptive_robot.utils.math_utils import rotations_to_meters, rps_to_mps

from config.constants import RobotConst, DrivetrainConst


class DrivetrainIO(ABC):
    """
    Blueprint of the required methods for a drivetrain IO.
    An IO holds all of the getters and setters to directly communicate with hardware.
    """
    @abstractmethod
    def get_left_voltage(self) -> volts: 
        """
        Returns the applied voltage to the front left motor.
        """
        ...
    
    @abstractmethod
    def get_right_voltage(self) -> volts: 
        """
        Returns the applied voltage to the front right motor.
        """
        ...

    @abstractmethod
    def get_left_distance(self) -> meters: 
        """
        Returns the total distance traveled by the left motor in meters.
        """
        ...
    
    @abstractmethod
    def get_right_distance(self) -> meters: 
        """
        Returns the total distance traveled by the right motor in meters.
        """
        ...
    
    @abstractmethod
    def get_left_velocity(self) -> meters_per_second: 
        """
        Returns the current velocity of the left motor in MPS.
        """
        ...
    
    @abstractmethod
    def get_right_velocity(self) -> meters_per_second: 
        """
        Returns the current velocity of the right motor in MPS.
        """
        ...
    
    @abstractmethod
    def get_heading(self) -> Rotation2d: 
        """
        Returns the gyro-measured drivetrain angle.
        """
        ...
    
    @abstractmethod
    def command_left_voltage(self, volts: volts) -> None: 
        """
        Commands a voltage to the left motors.
        """
        ...
    
    @abstractmethod
    def command_right_voltage(self, volts: volts) -> None: 
        """
        Commands a voltage to the right motors.
        """
        ...

    @abstractmethod
    def set_left_position(self, position: float) -> None: 
        """
        Sets the position of the left encoder in rotations.
        """
        ...

    @abstractmethod
    def set_right_position(self, position: float) -> None: 
        """
        Sets the position of the right encoder in rotations.
        """
        ...

    @abstractmethod
    def reset_gyro(self) -> None: 
        """
        Resets the gyro readings to zero.
        """
        ...

    @abstractmethod
    def update(self) -> None:
        """
        Updates the state of the IO, if needed.
        """
        ...


class RealDrivetrainIO(DrivetrainIO):
    """
    Contains all setters and getters for the actual hardware of the bot.
    """
    def __init__(self) -> None:
        self.front_left_motor = TalonSRX(1)
        self.front_right_motor = TalonSRX(2)
        self.back_left_motor = TalonSRX(3)
        self.back_right_motor = TalonSRX(4)

        self.left_encoder = AdaptiveDutyCycleEncoder(0)
        self.right_encoder = AdaptiveDutyCycleEncoder(1)

        self.gyro = AHRS.create_spi()

        self.front_right_motor.setInverted(True)
        self.back_right_motor.setInverted(True)

        self.back_left_motor.follow(self.front_left_motor)
        self.back_right_motor.follow(self.front_right_motor)

        for motor in (
            self.front_left_motor, self.front_right_motor, self.back_left_motor, self.back_right_motor
        ):
            motor.enableVoltageCompensation(True)
            motor.configVoltageCompSaturation(RobotConst.NOMINAL_VOLTAGE)
            motor.setNeutralMode(NeutralMode.Brake)

            motor.configContinuousCurrentLimit(DrivetrainConst.SRX_CONTINUOUS_AMPERE_LIMIT)
            motor.configPeakCurrentLimit(DrivetrainConst.SRX_PEAK_AMPERE_LIMIT)
            motor.configPeakCurrentDuration(DrivetrainConst.SRX_PEAK_DURATION_MS)
            motor.enableCurrentLimit(True)

    def get_left_voltage(self) -> volts:
        return self.front_left_motor.getMotorOutputVoltage()

    def get_right_voltage(self) -> volts:
        return self.front_right_motor.getMotorOutputVoltage()

    def get_left_distance(self) -> meters:
        return rotations_to_meters(
            motor_rotations=self.left_encoder.get_position(),
            wheel_diameter=DrivetrainConst.WHEEL_DIAMETER,
            gear_ratio=DrivetrainConst.GEAR_RATIO
        )

    def get_right_distance(self) -> meters:
        return rotations_to_meters(
            motor_rotations=self.right_encoder.get_position(),
            wheel_diameter=DrivetrainConst.WHEEL_DIAMETER,
            gear_ratio=DrivetrainConst.GEAR_RATIO
        )

    def get_left_velocity(self) -> meters_per_second:
        return rps_to_mps(
            motor_rps=self.left_encoder.get_velocity(),
            wheel_diameter=DrivetrainConst.WHEEL_DIAMETER,
            gear_ratio=DrivetrainConst.GEAR_RATIO
        )

    def get_right_velocity(self) -> meters_per_second:
        return rps_to_mps(
            motor_rps=self.right_encoder.get_velocity(),
            wheel_diameter=DrivetrainConst.WHEEL_DIAMETER,
            gear_ratio=DrivetrainConst.GEAR_RATIO
        )

    def get_heading(self) -> Rotation2d:
        return self.gyro.getRotation2d()
    
    def command_left_voltage(self, volts: volts) -> None:
        percent = volts / RobotConst.NOMINAL_VOLTAGE
        self.front_left_motor.set(TalonSRXControlMode.PercentOutput, percent)
    
    def command_right_voltage(self, volts: volts) -> None:
        percent = volts / RobotConst.NOMINAL_VOLTAGE
        self.front_right_motor.set(TalonSRXControlMode.PercentOutput, percent)
    
    def set_left_position(self, position: float) -> None:
        self.left_encoder.set_position(position)
    
    def set_right_position(self, position: float) -> None:
        self.right_encoder.set_position(position)
    
    def reset_gyro(self) -> None:
        self.gyro.reset()

    def update(self) -> None:
        pass


class SimDrivetrainIO(DrivetrainIO):
    """
    Mocks real drivetrain IO logic with physics for sim.
    """
    def __init__(self) -> None:
        self.left_volts = 0.0
        self.right_volts = 0.0

        self.left_position_offset = 0.0
        self.right_position_offset = 0.0

        self.diff_sim = DifferentialDrivetrainSim(
            driveMotor=DCMotor.NEO(2),
            gearing=DrivetrainConst.GEAR_RATIO,
            J=DrivetrainConst.MOI,
            mass=RobotConst.MASS,
            wheelRadius=DrivetrainConst.WHEEL_DIAMETER / 2.0,
            trackWidth=DrivetrainConst.TRACK_WIDTH
        )

    def get_left_voltage(self) -> volts:
        return self.left_volts

    def get_right_voltage(self) -> volts:
        return self.right_volts
    
    def get_left_distance(self) -> meters:
        return self.diff_sim.getLeftPosition() + self.left_position_offset
    
    def get_right_distance(self) -> meters:
        return self.diff_sim.getRightPosition() + self.right_position_offset
    
    def get_left_velocity(self) -> meters_per_second:
        return self.diff_sim.getLeftVelocity()
    
    def get_right_velocity(self) -> meters_per_second:
        return self.diff_sim.getRightVelocity()

    def get_heading(self) -> Rotation2d:
        return self.diff_sim.getHeading()
    
    def command_left_voltage(self, volts: volts) -> None:
        self.left_volts = volts

    def command_right_voltage(self, volts: volts) -> None:
        self.right_volts = volts

    def set_left_position(self, position: float) -> None:
        self.left_position_offset = position - self.diff_sim.getLeftPosition()

    def set_right_position(self, position: float) -> None:
        self.right_position_offset = position - self.diff_sim.getRightPosition()

    def reset_gyro(self) -> None:
        pose = self.diff_sim.getPose()
        self.diff_sim.setPose(Pose2d(pose.X(), pose.Y(), 0.0))

    def update(self) -> None:
        self.diff_sim.setInputs(self.left_volts, self.right_volts)
        self.diff_sim.update(RobotConst.LOOP_DT)


class FakeDrivetrainIO(DrivetrainIO):
    """
    Mocks real drivetrain IO logic for unit testing.
    """
    def __init__(self) -> None:
        self.left_voltage = 0.0
        self.right_voltage = 0.0

        self.left_distance = 0.0
        self.right_distance = 0.0

        self.left_velocity = 0.0
        self.right_velocity = 0.0

        self.heading = Rotation2d()

    def get_left_voltage(self) -> volts:
        return self.left_voltage

    def get_right_voltage(self) -> volts:
        return self.right_voltage

    def get_left_distance(self) -> meters:
        return self.left_distance

    def get_right_distance(self) -> meters:
        return self.right_distance

    def get_left_velocity(self) -> meters_per_second:
        return self.left_velocity

    def get_right_velocity(self) -> meters_per_second:
        return self.right_velocity

    def get_heading(self) -> Rotation2d:
        return self.heading

    def command_left_voltage(self, volts: volts) -> None:
        self.left_voltage = volts

    def command_right_voltage(self, volts: volts) -> None:
        self.right_voltage = volts

    def set_left_position(self, position: float) -> None:
        self.left_distance = position

    def set_right_position(self, position: float) -> None:
        self.right_distance = position

    def reset_gyro(self) -> None:
        self.heading = Rotation2d()

    def update(self) -> None:
        pass

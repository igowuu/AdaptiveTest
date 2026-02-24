from abc import ABC, abstractmethod

from phoenix5 import TalonSRX, TalonSRXControlMode

from wpimath.units import radians, radians_per_second, volts

from adaptive_robot.hardware.adaptive_dutycycle_encoder import AdaptiveDutyCycleEncoder
from adaptive_robot.utils.math_utils import rotations_to_radians, rps_to_radps

from config.constants import RobotConst, ArmConst


class ArmIO(ABC):
    """
    Blueprint of the required methods for an arm IO.
    An IO holds all of the getters and setters to directly communicate with hardware.
    """
    @abstractmethod
    def get_voltage(self) -> volts:
        """
        Returns the applied voltage to the arm.
        """
        ...

    @abstractmethod
    def get_angle(self) -> radians:
        """
        Returns the arm angle in radians.
        """
        ...

    @abstractmethod
    def get_velocity(self) -> radians_per_second:
        """
        Returns the current velocity in radians per second.
        """
        ...

    @abstractmethod
    def command_voltage(self, volts: volts) -> None: 
        """
        Commands a voltage to the motors.
        """
        ...

    @abstractmethod
    def update(self) -> None:
        """
        Updates the state of the IO.
        """
        ...


class RealArmIO(ArmIO):
    def __init__(self) -> None:
        self.left_arm_motor = TalonSRX(21)
        self.right_arm_motor = TalonSRX(22)

        self.right_arm_motor.setInverted(True)

        self.right_arm_motor.follow(self.left_arm_motor)

        self.left_arm_motor.enableVoltageCompensation(True)
        self.left_arm_motor.configVoltageCompSaturation(RobotConst.NOMINAL_VOLTAGE)

        self.right_arm_motor.enableVoltageCompensation(True)
        self.right_arm_motor.configVoltageCompSaturation(RobotConst.NOMINAL_VOLTAGE)

        self.left_arm_encoder = AdaptiveDutyCycleEncoder(2)
        self.right_arm_encoder = AdaptiveDutyCycleEncoder(3)
    
    def get_voltage(self) -> radians:
        return self.left_arm_motor.getMotorOutputVoltage()

    def get_angle(self) -> radians:
        return rotations_to_radians(
            motor_rotations=self.left_arm_encoder.get_position(),
            gear_ratio=ArmConst.GEAR_RATIO
        )
    
    def get_velocity(self) -> radians_per_second:
        return rps_to_radps(
            motor_rps=self.left_arm_encoder.get_velocity(),
            gear_ratio=ArmConst.GEAR_RATIO
        )
    
    def command_voltage(self, volts: volts) -> None:
        percent = volts / RobotConst.NOMINAL_VOLTAGE
        self.left_arm_motor.set(TalonSRXControlMode.PercentOutput, percent)

    def update(self) -> None:
        pass


class SimArmIO:
    pass


class FakeArmIO:
    pass

from abc import ABC, abstractmethod

from phoenix5 import TalonSRX, TalonSRXControlMode

from wpilib.simulation import SingleJointedArmSim

from wpimath.units import radians, radians_per_second, volts
from wpimath.system.plant import DCMotor

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
        self.left_arm_motor = TalonSRX(5)
        self.right_arm_motor = TalonSRX(6)

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


class SimArmIO(ArmIO):
    def __init__(self) -> None:
        self.voltage = 0.0

        self.arm_sim = SingleJointedArmSim(
            gearbox=DCMotor.CIM(2),
            gearing=ArmConst.GEAR_RATIO,
            moi=ArmConst.MOI,
            armLength=ArmConst.LENGTH,
            minAngle=ArmConst.SIM_MIN_ANGLE,
            maxAngle=ArmConst.SIM_MAX_ANGLE,
            simulateGravity=True,
            startingAngle=ArmConst.SIM_MIN_ANGLE
        )

    def get_voltage(self) -> volts:
        return self.voltage

    def get_angle(self) -> radians:
        return self.arm_sim.getAngle()

    def get_velocity(self) -> radians_per_second:
        return self.arm_sim.getVelocity()
    
    def command_voltage(self, volts: volts) -> None:
        self.voltage = volts

    def update(self) -> None:
        self.arm_sim.setInputVoltage(self.voltage)
        self.arm_sim.update(RobotConst.LOOP_DT)


class FakeArmIO:
    def __init__(self) -> None:
        self.voltage = 0.0
        self.angle = 0.0
        self.velocity = 0.0

    def get_voltage(self) -> volts:
        return self.voltage

    def get_angle(self) -> radians:
        return self.angle

    def get_velocity(self) -> radians_per_second:
        return self.voltage

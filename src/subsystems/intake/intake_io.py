from abc import ABC

from phoenix6.hardware import TalonFX

from wpimath.units import volts


class IntakeIO(ABC):
    def get_voltage(self) -> volts:
        """
        Returns the applied voltage to the intake.
        """
        ...
    
    def command_voltage(self, volts: volts) -> None:
        """
        Commands a voltage to the intake.
        """
        ...
    

class RealIntakeIO(IntakeIO):
    def __init__(self) -> None:
        self.motor = TalonFX(7)

    def get_voltage(self) -> volts:
        return self.motor.get_motor_voltage().value
    
    def command_voltage(self, volts: volts) -> None:
        self.motor.setVoltage(volts)


class SimIntakeIO(IntakeIO):
    def __init__(self) -> None:
        pass

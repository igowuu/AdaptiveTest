import math

from wpimath.units import (
    meters, 
    radians,
    kilograms, 
    kilogram_square_meters,
    meters_per_second,
    radians_per_second,
    percent,
    volts,
    volt_seconds_per_meter,
    volt_seconds_squared_per_meter,
    seconds
)


class RobotConst:
    MASS: kilograms = 45.3

    MAX_PCT_OUT: percent = 1.0
    NOMINAL_VOLTAGE: volts = 12.0

    LOOP_DT: seconds = 0.02


class DrivetrainConst:
    GEAR_RATIO = 10.71

    MAX_SPEED_MPS: meters_per_second = 4.229
    MAX_SPEED_RADPS: radians_per_second = 10.703

    WHEEL_DIAMETER: meters = 0.1524
    TRACK_WIDTH: meters = 0.6

    MOI: kilogram_square_meters = 6.0

    SLEW_LIMIT: volts = 36

    SRX_CONTINUOUS_AMPERE_LIMIT = 32
    SRX_PEAK_AMPERE_LIMIT = 45
    SRX_PEAK_DURATION_MS = 150


class DrivetrainPID:
    LEFT_KP = 0.01
    LEFT_KI = 0.0
    LEFT_KD = 0.0

    RIGHT_KP = 0.01
    RIGHT_KI = 0.0
    RIGHT_KD = 0.0


class DrivetrainFF:
    LEFT_KS: volts = 0.0
    LEFT_KV: volt_seconds_per_meter = 2.5
    LEFT_KA: volt_seconds_squared_per_meter = 0.5

    RIGHT_KS: volts = 0.0
    RIGHT_KV: volt_seconds_per_meter = 2.5
    RIGHT_KA: volt_seconds_squared_per_meter = 0.5


class ArmConst:
    GEAR_RATIO = 30.0  # motor rotations per arm rotation
    MOI: kilogram_square_meters = 0.1  # Moment of inertia of arm (estimate)
    LENGTH: meters = 0.2  # Arm length

    MIN_ANGLE: radians = 0.0
    MAX_ANGLE: radians = math.pi / 2.0

    APPLIED_PCT_OUT: percent = 1.0
    MAX_RADPS: radians_per_second = 6.3

    # Simulation constants (unitless, all in terms of the GUI)
    SIM_HEIGHT = 50.0
    SIM_WIDTH = 20.0
    ROOT_LENGTH = 10.0
    ROOT_ANGLE = 0.0
    ROOT_X = 10.0
    ROOT_Y = 1.0


class ArmVelocityPID:
    # Arm velocity PID (input in radps, output in volts - untuned)
    KP = 0.01
    KI = 0.0
    KD = 0.0


# Arm PID
class ArmPositionPID:
    # Arm position PID (input in radps, output in volts - untuned)
    KP = 0.01
    KI = 0.0
    KD = 0.0


class ArmFF:
    # Arm FF (estimates)
    KS: volts = 0.50
    KG: volts = 0.0
    KV: volt_seconds_per_meter = 2.60
    KA: volt_seconds_squared_per_meter = 0.05

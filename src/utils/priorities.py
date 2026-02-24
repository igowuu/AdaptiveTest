from enum import Enum

class DrivetrainPriority(Enum):
    """
    Determines which types of requests are able to override other requests.
    Higher values represent higher priorities.
    For example, SAFETY requests would override all other requests.
    """
    SAFETY = 4
    AUTO = 3
    TELEOP = 2
    SYSID = 1
    TEST = 0

class ArmPriority(Enum):
    SAFETY = 4
    AUTO = 3
    TELEOP = 2
    SYSID = 1
    TEST = 0

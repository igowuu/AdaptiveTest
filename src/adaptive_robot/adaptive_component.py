from typing import final, Any, TYPE_CHECKING
from abc import abstractmethod, ABC
import logging

from adaptive_robot.tunable.tunable_value import TunableValue, TunableType
from adaptive_robot.tunable.tunable_pid_controller import TunablePIDController
from adaptive_robot.telemetry import primitive_type

if TYPE_CHECKING:
    from adaptive_robot.adaptive_robot import AdaptiveRobot


logger = logging.getLogger(__name__)


class AdaptiveComponent(ABC):
    """
    This base class defines the execution and telemetry lifecycle for all robot components.
    Subclasses should implement execute() and may override publish_telemetry().
    """
    def __init__(self, robot: "AdaptiveRobot", name: str) -> None:
        """
        Creates an AdaptiveComponent object to register into the scheduler.

        :param robot: The parent robot object to attatch the component to.
        :param name: The name assigned to the component, used for safety logging in NetworkTables.
        """
        self._robot = robot
        self._robot.add_component(self)

        self.name = name

        self._tunables: list[TunableValue[Any]] = []
        self._tunable_pids: list[TunablePIDController] = []

        self._failure_count_threshold = 5
        self._failure_count = 0
        self._is_healthy = True

    @final
    def update_tunable_constants(self) -> None:
        """
        Allows the scheduler to update all tunable constants for the component.

        The user should not call this method.
        """
        for tunable in self._tunables:
            tunable.update()

    @final
    def update_tunable_pids(self) -> None:
        """
        Allows the scheduler to update all tunable PID objects for the component.

        The user should not call this method.
        """
        for tunable_pid in self._tunable_pids:
            tunable_pid.update_from_tunables()

    @final
    def record_failure(self, exception: Exception) -> None:
        """
        Allows the scheduler to record an exception raised by the component.
        If the amount of subsequent exceptions exceeds the threshold, set component to unhealthy.

        The user should not call this method.
        """
        logger.error(f"{self.name}: {exception}")

        self._failure_count += 1

        if self._failure_count > self._failure_count_threshold:
            self._is_healthy = False
            logger.info(f"{self._failure_count} consecutive failures raised. {self.name} has been disabled.")
    
    @final
    def record_safety_telemetry(self) -> None:
        """
        Allows the scheduler to record component-specific safety data each iteration.

        The user should not call this method.
        """
        self.publish_value(f"Safety/{self.name}", self.is_healthy())

    @final
    def reset_failure_count(self) -> None:
        """
        Allows the scheduler to reset the count of subsequent failures made.

        The user should not call this method.
        """
        self._failure_count = 0

    @final
    def is_healthy(self) -> bool:
        """
        Returns True if no faults within the component have been detected.
        """
        return self._is_healthy

    @final
    def tunable(self, directory: str, default: TunableType) -> TunableValue[Any]:
        """
        Creates an object that can be altered through NetworkTables and will update the variable
        in the codebase.

        TunableValue does not inherently change any values (including objects) previously 
        made with the value.
        
        :param directory: The directory that the value will be saved under in NetworkTables.
        :param default: The default value published at runtime.
        """
        value = TunableValue(directory, default)
        self._tunables.append(value)
        return value
    
    @final
    def tunablePID(
        self,
        kp: float,
        ki: float,
        kd: float,
        directory: str = "Tunables/PIDController",
        period: float = 0.02
    ) -> TunablePIDController:
        """
        Creates an object that can be altered through NetworkTables and will update a TunablePIDController
        object every iteration in the codebase.
        
        :param kp: The proportional coefficient. Must be >= 0.
        :param ki: The integral coefficient. Must be >= 0.
        :param kd: The derivative coefficient. Must be >= 0.
        :param period: The period between controller updates in seconds. The default is 20 milliseconds. Must be positive.
        :param directory: The directory in NetworkTables in which the tunable PID values will be located at.
        """
        tunablePID = TunablePIDController(
            kp=TunableValue(f"{directory}/kp", kp),
            ki=TunableValue(f"{directory}/ki", ki), 
            kd=TunableValue(f"{directory}/kd", kd),
            period=period
        )
        self._tunable_pids.append(tunablePID)
        return tunablePID
    
    @final
    def publish_value(self, directory: str, value: primitive_type) -> None:
        """
        Publishes an immutable value to networktables.
        
        :param directory: The directory that the value will be saved under in NetworkTables.
        :param value: The value you wish to save publish to NetworkTables.
        """
        self._robot.telemetry_publisher.put_value(directory, value)
    
    def on_faulted(self) -> None:
        """
        Method that is called each iteration if the component is no longer healthy.
        """
        ...

    def publish_telemetry(self) -> None:
        """
        Method automatically called every loop in the program before execute().
        """
        ...
    
    @abstractmethod
    def execute(self) -> None: 
        """
        Method automatically called every loop in the program.
        """
        ...

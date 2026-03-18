from dataclasses import dataclass
from typing import Any, Optional

import wpilib

from ntcore import NetworkTableInstance, StructPublisher, StructSubscriber


@dataclass
class StructEntry:
    """
    Holds a struct publisher, subscriber, and its cached value for change detection.
    """
    publisher: StructPublisher
    subscriber: StructSubscriber
    cached_value: Any


class TelemetryStructPublisher:
    """
    Publishes WPILib struct types (Pose3d, Pose2d, etc.) through NetworkTables.
    Uses caching to significantly help with excess process speed.
    """
    def __init__(self) -> None:
        self._entries: dict[str, StructEntry] = {}

    def put_struct_value(self, directory: str, value: object) -> None:
        """
        Publishes a WPILib struct-compatible value to NetworkTables.

        :param directory: NetworkTables entry key (path). Use forward slashes for folders.
        :param value: The struct instance to publish (i.e. Pose3d(3, 4, 5)).
        """
        try:
            if directory not in self._entries:
                value_type = type(value)
                publisher = NetworkTableInstance.getDefault() \
                    .getStructTopic("Dashboard/" + directory, value_type) \
                    .publish()
                subscriber = publisher.getTopic().subscribe(value_type())
                self._entries[directory] = StructEntry(publisher, subscriber, value)

                # Set value immediately on first iteration 
                publisher.set(value)
            else:
                entry = self._entries[directory]
                # Only publish if value has changed
                if entry.cached_value != value:
                    entry.publisher.set(value)
                    entry.cached_value = value

        except Exception as e:
            wpilib.reportError(
                error=f"Struct publish failed for directory='{directory}': {e}",
                printTrace=False
            )

    def get_struct_value(self, key: str) -> Optional[Any]:
        """
        Returns the value in networktables for a given struct key.
        """
        return self._entries[key].subscriber.get()

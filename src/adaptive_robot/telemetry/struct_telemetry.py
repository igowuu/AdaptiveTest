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

    def put_struct_value(self, key: str, struct_type: type, value: Any) -> None:
        """
        Publishes a WPILib struct-compatible value to NetworkTables.

        :param key: NetworkTables entry key (path). Use forward slashes for folders.
        :param struct_type: The struct type class (e.g., Pose3d, Pose2d).
        :param value: The struct instance to publish.
        """
        try:
            if key not in self._entries:
                publisher = NetworkTableInstance.getDefault() \
                    .getStructTopic(key, struct_type) \
                    .publish()
                subscriber = publisher.getTopic().subscribe(struct_type())
                self._entries[key] = StructEntry(publisher, subscriber, struct_type())

            entry = self._entries[key]

            # Only publish if value has changed
            if entry.cached_value != value:
                entry.publisher.set(value)
                entry.cached_value = value

        except Exception as e:
            wpilib.reportError(
                error=f"Struct publish failed for key='{key}': {e}",
                printTrace=False
            )

    def get_struct_value(self, key: str) -> Optional[Any]:
        if 
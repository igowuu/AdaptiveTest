from abc import ABC, abstractmethod
from enum import Enum, auto
from typing import final


class ActionState(Enum):
    """
    Represents the lifecycle state of an action.
    """
    READY = auto()
    RUNNING = auto()
    FINISHED = auto()
    CANCELED = auto()


class Action(ABC):
    """
    Represents a single action for autonomous.
    An action manages its own state and determines when it is finished.
    """
    def __init__(self) -> None:
        self._state = ActionState.READY

    @final
    def set_running(self) -> None:
        """
        Sets the current state to running.
        """
        self._state = ActionState.RUNNING

    @final
    def set_finished(self) -> None:
        """
        Sets the current state to finished if previously running.
        """
        if self._state == ActionState.RUNNING:
            self._state = ActionState.FINISHED
        else:
            raise RuntimeError("Attempted to set an action to a finished state that was not running.")
    
    @final
    def set_canceled(self) -> None:
        """
        Sets the current state to finished if previously running.
        """
        if self._state == ActionState.RUNNING:
            self._state = ActionState.CANCELED
        else:
            raise RuntimeError("Attempted to set an action to a canceled state that was not running.")

    @final
    @property
    def state(self) -> ActionState:
        """
        Returns the current state of this action.
        """
        return self._state

    @final
    @property
    def is_finished(self) -> bool:
        """
        Returns True if the action is no longer running.
        """
        return self._state in (ActionState.FINISHED, ActionState.CANCELED)

    @final
    def start(self) -> None:
        """
        Initializes and starts the action.
        Called once when the action begins execution.
        """
        if self._state != ActionState.READY:
            raise RuntimeError("Attempted to start an autonomous action that was not ready.")

        self.set_running()
        should_finish = self.on_start()

        if should_finish:
            self.set_finished()
            self.on_finish()

    @final
    def update(self) -> None:
        """
        Executes one iteration of the action.
        Called repeatedly until the action finishes.
        """
        if self._state != ActionState.RUNNING:
            raise RuntimeError("Attempted to update an autonomous action that was not running.")

        should_finish = self.on_update()

        if should_finish:
            self.set_finished()
            self.on_finish()

    @final
    def cancel(self) -> None:
        """
        Immediately terminates the action.
        Called when the action needs to stop before completion.
        """
        if self._state != ActionState.RUNNING:
            raise RuntimeError("Attempted to cancel an autonomous action that was not running.")

        self.on_cancel()
        self.set_canceled()

    @abstractmethod
    def on_start(self) -> bool:
        pass

    @abstractmethod
    def on_update(self) -> bool:
        pass

    @abstractmethod
    def on_cancel(self) -> None:
        pass

    @abstractmethod
    def on_finish(self) -> None:
        pass


class SequentialAction(Action):
    def __init__(self, actions: list[Action]) -> None:
        super().__init__()

        self._actions = actions
        self._current_index = 0

    def _start_current_action(self) -> None:
        """
        Starts the current action if it is ready.
        """
        if self._current_index >= len(self._actions):
            return

        current_action = self._actions[self._current_index]
        if current_action.state == ActionState.READY:
            current_action.start()

    def on_start(self) -> bool:
        """
        Starts the first action in the provided list.
        Returns if the action should be terminated upon an empty list.
        """
        if not self._actions:
            return True

        self._start_current_action()
        current_action = self._actions[self._current_index]

        if current_action.is_finished:
            self._current_index += 1
            if self._current_index >= len(self._actions):
                return True
            self._start_current_action()

        return False
    
    def on_update(self) -> bool:
        """
        Updates the current action each iteration.
        If the current action finishes this iteration, progresses to the next action.
        If all actions have been completed, sets the SequentialAction to finished.
        """
        if self._current_index >= len(self._actions):
            return True

        current_action = self._actions[self._current_index]
        if current_action.state == ActionState.READY:
            current_action.start()

        if current_action.state == ActionState.RUNNING:
            current_action.update()

        if current_action.is_finished:
            self._current_index += 1
            if self._current_index >= len(self._actions):
                return True

            self._start_current_action()

        return False

    def on_cancel(self) -> None:
        """
        Cancels both the current action and the SequentialAction.
        """
        if self._current_index < len(self._actions):
            current_action = self._actions[self._current_index]
            if current_action.state == ActionState.RUNNING:
                current_action.cancel()

    def on_finish(self) -> None:
        pass

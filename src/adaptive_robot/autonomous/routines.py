from abc import ABC, abstractmethod 
from enum import Enum, auto


class ActionState(Enum):
    READY = auto()
    RUNNING = auto()
    FINISHED = auto()
    CANCELED = auto()


class Action(ABC):
    """
    Represents a single autonomous action that contains the required lifecycle methods.
    """
    @abstractmethod
    def start(self) -> None:
        """
        Starts and initializes an action.
        """
        pass

    @abstractmethod
    def update(self) -> None:
        """
        Updates the action each iteration.
        """
        pass

    @abstractmethod
    def end(self) -> None:
        """
        Ends the action once it naturally finishes.
        """
        pass
    
    def cancel(self) -> None:
        """
        Immediately terminates the execution of an action once called.
        """
        ...

    def reset(self) -> None:
        """
        Resets the action so it can be run again.
        """
        ...

    def get_state(self) -> ActionState:
        """
        Returns the currently active state type.
        """
        ...

    @property
    @abstractmethod
    def is_finished(self) -> bool:
        """
        Returns True if the action is complete.
        """
        pass


class SequentialAction(Action):
    """
    Initializes a sequential action, where each action waits for the previous to end before executing.
    """
    def __init__(self, actions: list[Action]) -> None:
        """
        Given a list of actions, initializes a sequential action.
        Each action will be executed in ascending order in the list.
        """
        self._actions = actions
        self._current_index = 0
        self._state = ActionState.READY

    def start(self) -> None:
        """
        Starts the first action in the provided list.
        """
        if not self._state == ActionState.READY:
            return

        self._current_index = 0
        if self._actions:
            self._actions[0].start()

        self._state = ActionState.RUNNING

    def update(self) -> None:
        """
        Updates the action each iteration if not all actions have been completed.
        If the currently running action has ended, progresses to the next action.
        """
        if not self._state == ActionState.RUNNING:
            return

        if self._current_index >= len(self._actions):
            return

        current_action = self._actions[self._current_index]
        current_action.update()

        if current_action.is_finished:
            current_action.end()
            self._current_index += 1
            if self._current_index < len(self._actions):
                self._actions[self._current_index].start()

    def end(self) -> None:
        """
        Stops the current action if still running.
        """
        if not self._state == ActionState.RUNNING:
            return

        if self._current_index < len(self._actions):
            self._actions[self._current_index].end()

        self._state = ActionState.FINISHED

    def cancel(self) -> None:
        if self._state == ActionState.RUNNING:
            for action in self._actions:
                if action.get_state() == ActionState.RUNNING:
                    action.cancel()
            self._state = ActionState.CANCELED
            self.end()

    def reset(self) -> None:
        if self._state in (ActionState.FINISHED, ActionState.CANCELED):
            self._state = ActionState.READY
            self._current_index = 0
            for action in self._actions:
                action.reset()

    def get_state(self) -> ActionState:
        return self._state

    @property
    def is_finished(self) -> bool:
        """
        Returns True if all actions have been completed.
        """
        return self._state in (ActionState.FINISHED, ActionState.CANCELED)


class ParallelAction(Action):
    """
    Initializes a parallel action, where each action runs at the same time.
    The action will end once all actions have finished.
    """
    def __init__(self, actions: list[Action]) -> None:
        """
        Given a list of actions, initializes a parallel action.
        """
        self._actions = actions
        self._active_actions: list[Action] = []
        self._state = ActionState.READY

    def start(self) -> None:
        """
        Calls the start method for all actions.
        """
        if not self._state == ActionState.READY:
            return

        self._active_actions = list(self._actions)
        for action in self._active_actions:
            action.start()

        self._state = ActionState.RUNNING

    def update(self) -> None:
        """
        Updates all actions that haven't finished.
        """
        if not self._state == ActionState.RUNNING:
            return

        still_active: list[Action] = []

        for action in self._active_actions:
            action.update()
            if action.is_finished:
                action.end()
            else:
                still_active.append(action)

        self._active_actions = still_active

    def end(self) -> None:
        """
        Stops all running actions.
        """
        if not self._state == ActionState.RUNNING:
            return

        for action in self._active_actions:
            action.end()
        self._active_actions.clear()

        self._state = ActionState.FINISHED

    def cancel(self) -> None:
        if self._state == ActionState.RUNNING:
            for action in self._actions:
                if action.get_state() == ActionState.RUNNING:
                    action.cancel()
            self._state = ActionState.CANCELED
            self.end()

    def reset(self) -> None:
        if self._state in (ActionState.FINISHED, ActionState.CANCELED):
            self._state = ActionState.READY
            self._active_actions.clear()
            for action in self._actions:
                action.reset()

    def get_state(self) -> ActionState:
        return self._state

    @property
    def is_finished(self) -> bool:
        """
        Returns True if there are no more active actions.
        """
        return self._state in (ActionState.FINISHED, ActionState.CANCELED)

import pytest

from adaptive_robot.autonomous.routines import Action, ActionState, SequentialAction


class FakeAction(Action):
	def __init__(
		self, 
		finish_on_start: bool = False, 
		finish_after_updates: int | None = None
	) -> None:
		super().__init__()
		self.finish_on_start = finish_on_start
		self.finish_after_updates = finish_after_updates
		self.start_calls = 0
		self.update_calls = 0
		self.cancel_calls = 0
		self.finish_calls = 0

	def on_start(self) -> bool:
		self.start_calls += 1
		return self.finish_on_start

	def on_update(self) -> bool:
		self.update_calls += 1
		if self.finish_after_updates is None:
			return False
		return self.update_calls >= self.finish_after_updates

	def on_cancel(self) -> None:
		self.cancel_calls += 1

	def on_finish(self) -> None:
		self.finish_calls += 1


def test_action_start_transitions_to_running_and_calls_on_start_once() -> None:
	action = FakeAction()

	action.start()

	assert action.state == ActionState.RUNNING
	assert action.start_calls == 1
	assert action.finish_calls == 0


def test_action_start_can_finish_immediately() -> None:
	action = FakeAction(finish_on_start=True)

	action.start()

	assert action.state == ActionState.FINISHED
	assert action.start_calls == 1
	assert action.finish_calls == 1


def test_action_update_calls_on_update_and_finishes_when_requested() -> None:
	action = FakeAction(finish_after_updates=2)
	action.start()

	action.update()
	assert action.state == ActionState.RUNNING
	assert action.update_calls == 1
	assert action.finish_calls == 0

	action.update()
	assert action.state == ActionState.FINISHED
	assert action.update_calls == 2
	assert action.finish_calls == 1


def test_action_update_raises_if_not_running() -> None:
	action = FakeAction()

	with pytest.raises(RuntimeError):
		action.update()


def test_action_cancel_transitions_to_canceled_and_calls_on_cancel() -> None:
	action = FakeAction()
	action.start()

	action.cancel()

	assert action.state == ActionState.CANCELED
	assert action.cancel_calls == 1
	assert action.is_finished


def test_action_cancel_raises_if_not_running() -> None:
	action = FakeAction()

	with pytest.raises(RuntimeError):
		action.cancel()


def test_sequential_action_empty_list_finishes_on_start() -> None:
	routine = SequentialAction([])

	routine.start()

	assert routine.state == ActionState.FINISHED
	assert routine.is_finished


def test_sequential_action_runs_actions_in_order() -> None:
	first = FakeAction(finish_after_updates=1)
	second = FakeAction(finish_after_updates=2)
	routine = SequentialAction([first, second])

	routine.start()
	assert first.start_calls == 1
	assert second.start_calls == 0

	routine.update()
	assert first.state == ActionState.FINISHED
	assert second.start_calls == 1
	assert routine.state == ActionState.RUNNING

	routine.update()
	assert second.state == ActionState.RUNNING
	assert routine.state == ActionState.RUNNING

	routine.update()
	assert second.state == ActionState.FINISHED
	assert routine.state == ActionState.FINISHED
	assert routine.is_finished


def test_sequential_action_cancel_cancels_current_child() -> None:
	first = FakeAction(finish_after_updates=5)
	second = FakeAction(finish_after_updates=1)
	routine = SequentialAction([first, second])
	routine.start()

	routine.cancel()

	assert routine.state == ActionState.CANCELED
	assert first.cancel_calls == 1
	assert second.cancel_calls == 0


def test_sequential_action_skips_child_finished_on_start_at_beginning() -> None:
	first = FakeAction(finish_on_start=True)
	second = FakeAction(finish_after_updates=1)
	routine = SequentialAction([first, second])

	routine.start()

	assert first.state == ActionState.FINISHED
	assert first.start_calls == 1
	assert first.update_calls == 0
	assert second.start_calls == 1
	assert routine.state == ActionState.RUNNING

	routine.update()

	assert second.state == ActionState.FINISHED
	assert routine.state == ActionState.FINISHED


def test_sequential_action_skips_middle_child_finished_on_start() -> None:
	first = FakeAction(finish_after_updates=1)
	second = FakeAction(finish_on_start=True)
	third = FakeAction(finish_after_updates=1)
	routine = SequentialAction([first, second, third])

	routine.start()

	routine.update()

	assert first.state == ActionState.FINISHED
	assert second.state == ActionState.FINISHED
	assert second.start_calls == 1
	assert second.update_calls == 0
	assert third.start_calls == 0
	assert routine.state == ActionState.RUNNING

	routine.update()

	assert third.start_calls == 1
	assert third.state == ActionState.RUNNING
	assert routine.state == ActionState.RUNNING

	routine.update()

	assert third.state == ActionState.FINISHED
	assert routine.state == ActionState.FINISHED

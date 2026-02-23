from typing import Optional
import logging

from adaptive_robot.sysid.sysidtest import Mechanism, Config, SysIdTest, SysIdRoutineLog, TestType, Direction


logger = logging.getLogger(__name__)


class SysIdRoutine:
    """
    Holds and generates all possible tests, given a mechanism and config.
    """
    def __init__(
        self,
        mechanism: Mechanism,
        config: Config,
        name: str
    ) -> None:
        """
        Creates a SysId routine for a single mechanism.
        
        :param mechanism: Mechanism object with callables to move and obtain values from it.
        :param config: Config object with test-specific info.
        :param name: Name for the SysId routine.
        """
        self.mechanism = mechanism
        self.config = config
        self.logger = SysIdRoutineLog(name)

        self.quasistatic_forward = SysIdTest(
            direction=Direction.kForward, 
            config=self.config, 
            mechanism=self.mechanism, 
            test_type=TestType.QUASISTATIC,
            logger=self.logger
        )
        self.quasistatic_reverse = SysIdTest(
            direction=Direction.kReverse, 
            config=self.config, 
            mechanism=self.mechanism, 
            test_type=TestType.QUASISTATIC, 
            logger=self.logger
        )
        self.dynamic_forward = SysIdTest(
            direction=Direction.kForward, 
            config=self.config, 
            mechanism=self.mechanism, 
            test_type=TestType.DYNAMIC, 
            logger=self.logger
        )
        self.dynamic_reverse = SysIdTest(
            direction=Direction.kReverse, 
            config=self.config, 
            mechanism=self.mechanism, 
            test_type=TestType.DYNAMIC, 
            logger=self.logger
        )

        self._active_test: Optional[SysIdTest] = None

    def run_quasistatic_forward(self) -> None:
        """
        Runs the quasistatic forward test automatically.
        You must call step() each iteration to execute the test.
        """
        self._start_test(self.quasistatic_forward)

    def run_quasistatic_reverse(self) -> None:
        """
        Runs the quasistatic reverse test automatically.
        You must call step() each iteration to execute the test.
        """
        self._start_test(self.quasistatic_reverse)

    def run_dynamic_forward(self) -> None:
        """
        Runs the dynamic forward test automatically.
        You must call step() each iteration to execute the test.
        """
        self._start_test(self.dynamic_forward)

    def run_dynamic_reverse(self) -> None:
        """
        Runs the dynamic reverse test automatically.
        You must call step() each iteration to execute the test.
        """
        self._start_test(self.dynamic_reverse)
    
    def step(self) -> None:
        """
        Advance the active test one iteration. If no test is active test it will ignore the call.
        This will trap and stop on exceptions to ensure the mechanism is left safe.
        """
        if not self._active_test:
            return

        try:
            self._active_test.step()
            if not self._active_test.is_running():
                logger.info("SysId test finished.")
                self._active_test = None

        except Exception:
            logger.exception("Exception during SysId test; stopping current test.")
            try:
                if self._active_test:
                    self._active_test.stop()
            except Exception:
                logger.exception("Failed to stop active SysId test cleanly.")
            finally:
                self._active_test = None

    def stop(self) -> None:
        """
        Stop any running test immediately.
        """
        if self._active_test and self._active_test.is_running():
            logger.info("Stopping active SysId test.")
            self._active_test.stop()
        self._active_test = None

    def is_running(self) -> bool:
        """
        Returns True if a test is currently running.
        """
        return self._active_test is not None and self._active_test.is_running()

    def _start_test(self, test: SysIdTest) -> None:
        if self._active_test is test and test.is_running():
            logger.debug("Requested start of test that's already running; ignoring.")
            return

        if self._active_test and self._active_test.is_running():
            logger.info("Stopping currently running SysId test before starting new one.")
            self._active_test.stop()

        self._active_test = test
        logger.info(f"Starting SysId test: {test.test_type} {test.direction} ")
        test.start()

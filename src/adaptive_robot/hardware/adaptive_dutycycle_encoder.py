from wpilib import DutyCycleEncoder, Timer


class AdaptiveDutyCycleEncoder(DutyCycleEncoder):
    """
    Wrapper for a wpilib DutyCycleEncoder to allow for higher-level control.
    """
    def __init__(self, channel: int) -> None:
        """
        Construct a new DutyCycleEncoder on a specific channel.

        This has a fullRange of 1 and an expectedZero of 0.
        
        :param channel: the RIO port to attach to.
        """
        super().__init__(channel)

        self.offset = 0.0

        self.prev_time = Timer.getFPGATimestamp()
        self.prev_angle = self.get_position()

    def get_position(self) -> float:
        """
        Returns the total distance traveled in encoder rotations.

        :return: Total distance traveled in encoder rotations.
        """
        return self.get() + self.offset
        
    def get_velocity(self) -> float:
        """
        Returns the current velocity in rotations per second.
        
        :return: Velocity in rotations / sec.
        """
        now = Timer.getFPGATimestamp()
        angle = self.get_position()

        dt = now - self.prev_time
        if dt <= 0:
            return 0.0

        delta = angle - self.prev_angle

        # Unwrap
        if delta > 0.5:
            delta -= 1.0
        elif delta < -0.5:
            delta += 1.0

        self.prev_angle = angle
        self.prev_time = now

        return delta / dt

    def set_position(self, position: float) -> None:
        """
        Sets the encoder position to a given value in encoder rotations.
        """
        current = self.get()

        self.prev_angle = position
        self.prev_time = Timer.getFPGATimestamp()

        self.offset = position - current

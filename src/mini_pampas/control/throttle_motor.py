class ThrottleMotor:
    def __init__(self, motor):
        self._motor = motor

        self.stop()

    def run_forward(self, speed: float = 0.4729):
        """ Runs the Throttle motor on the given speed. The speed must be between 0.0 and 1.0. I none speed is given, the function uses 0.4729"""
        self._motor.forward(speed=max(0.0, min(speed, 1.0)))

    def stop(self):
        """ Stops the motor """
        self._motor.stop()

    def close(self):
        """ Turns the device off and frees the used pins """
        self._motor.close()

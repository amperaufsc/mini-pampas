class ThrottleMotor:
    def __init__(self, motor):
        self._motor = motor

        self.stop()

    def run_forward(self):
        self._motor.forward(speed=0.4729)

    def stop(self):
        self._motor.stop()

    def close(self):
        self._motor.close()

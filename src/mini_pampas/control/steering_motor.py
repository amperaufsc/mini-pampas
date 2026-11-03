class SteeringMotor:
    def __init__(self, servo):
        self._servo = servo
        self.set_to_initial_angle()

    def set_angle(self, angle):
        self._servo.angle = max(-25, min(angle, 25))

    def set_to_initial_angle(self):
        self._servo.mid()

    def close(self):
        self._servo.close()

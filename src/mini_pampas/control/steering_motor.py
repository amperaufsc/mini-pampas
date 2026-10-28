class SteeringMotor:
    def __init__(self, servo):
        self._servo = servo

    def set_angle(self, angle):
        servo._servo.angle = angle

    def set_to_initial_angle(self):
        servo._servo.mid()

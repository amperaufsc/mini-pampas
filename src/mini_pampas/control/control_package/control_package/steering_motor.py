class SteeringMotor:
    def __init__(self, servo):
        self._servo = servo
        self.set_to_initial_angle()

    def set_angle(self, angle):
        """ Set the angle of the steering motor. The angle must be between -25 and 25 """
        self._servo.angle = max(-25, min(angle, 25))

    def set_to_initial_angle(self):
        """ Sets the servo to the mid angle (0º) """
        self._servo.mid()

    def close(self):
        """ Turns the device off and frees the used pins """
        self._servo.close()

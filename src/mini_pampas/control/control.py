import pigpio

class Control:
    __STEERING_PIN = 5
    __STEERING_LIMIT_FREQUENCY = 20000

    def __init__(self):
        self.__pi = pigpio.pi()
        self.__steering = self.SteeringMotor(self.__STEERING_PIN, self.__pi, self.__STEERING_LIMIT_FREQUENCY)

    def setSteeringAngle(self, angle):
        self.__steering.setAngle(angle, self.__pi)

    def setSteeringNeutralAngle(self):
        self.setSteeringAngle(0)

    class SteeringMotor:
        __angle = 0

        def __init__(self, pin, pi, frequency):
            self.__pin = pin
            pi.set_PWM_frequency(pin, frequency)
            self.setNeutralAngle(pi)

        def setAngle(self, angle, pi):
            self.__angle = angle
            pi.set_servo_pulsewidth(self.__pin, self.angleToDutyCycle(angle))

        def setNeutralAngle(self, pi):
            self.setAngle(0, pi)

        def angleToDutyCycle(self, angle):
            return ((angle + 60)*1000)/120 + 1000


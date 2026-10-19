import pigpio

class Control:
    __THROTTLE_PIN = 4
    __THROTTLE_LIMIT_FREQUENCY = 20000

    def __init__(self):
        self.__pi = pigpio.pi()
        self.__throttle = self.ThrottleMotor(self.__THROTTLE_PIN, self.__pi, self.__THROTTLE_LIMIT_FREQUENCY)

    def turnOnThrottle(self):
        self.__throttle.turnOn(self.__pi)

    def turnOnThrottle(self):
        self.__throttle.turnOff(self.__pi)

    class ThrottleMotor:
        def __init__(self, pin, pi, frequency):
            self.__pin = pin
            pi.set_PWM_frequency(pin, frequency)

        def turnOn(self, pi):
            pi.set_PWM_dutycycle(self.__pin, 120)

        def turnOff(self, pi):
            pi.set_PWM_dutycycle(self.__pin, 0)

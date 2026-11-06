from gpiozero import AngularServo, Motor
from mini_pampas.control.steering_motor import SteeringMotor
from mini_pampas.control.throttle_motor import ThrottleMotor

class Control:
    self._SERVO_ANGLE_MIN = -25
    self._SERVO_ANGLE_MAX = 25
    self._SERVO_MIN_PULSE_WIDTH = 0.89/1000
    self._SERVO_MAX_PULSE_WIDTH = 1.89/1000

    def __init__(self, front_servo_pin, back_servo_pin, front_motor_forward_pin, front_motor_backward_pin, back_motor_forward_pin, back_motor_backward_pin):
        self._front_motor = ThrottleMotor(Motor(front_motor_forward_pin, front_motor_backward_pin))
        self._back_motor = ThrottleMotor(Motor(back_motor_forward_pin, back_motor_backward_pin))

        self._front_servo = SteeringMotor(AngularServo(front_servo_pin, min_angle=self._SERVO_ANGLE_MIN, max_angle=self._SERVO_ANGLE_MAX, min_pulse_width=self._SERVO_MIN_PULSE_WIDTH, max_pulse_width=self._SERVO_MAX_PULSE_WIDTH))
        self._back_servo = SteeringMotor(AngularServo(back_servo_pin, min_angle=self._SERVO_ANGLE_MIN, max_angle=self._SERVO_ANGLE_MAX, min_pulse_width=self._SERVO_MIN_PULSE_WIDTH, max_pulse_width=self._SERVO_MAX_PULSE_WIDTH))

    def stop_car(self):
        """ Stops the two throttle motors """
        self._front_motor.stop()
        self._back_motor.stop()

    def set_car_initial_direction(self):
        """ Set the angle of the two servos to the mid angle """
        self._front_servo.set_to_initial_angle()
        self._back_servo.set_to_initial_angle()

    def set_car_direction(self, direction):
        """ Set the angle of the front servo to the given direction """
        self._front_servo.set_angle(direction)
        self._back_servo.set_to_initial_angle()

    def run_forward(self):
        """ Runs the two throttle motors """
        self._front_motor.run_forward()
        self._back_motor.run_forward()

    def close(self):
        """ Turns all the motors off frees the pins """
        self.set_car_initial_direction()
        self.stop_car()
        self._front_servo.close()
        self._back_servo.close()
        self._front_motor.close()
        self._back_motor.close()

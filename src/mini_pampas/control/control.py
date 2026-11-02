from gpiozero import AngularServo, Motor
from steering_motor import SteeringMotor
from throttle_motor import ThrottleMotor

class Control:
    def __init__(self, front_servo_pin, back_servo_pin, front_motor_forward_pin, front_motor_backward_pin, back_motor_forward_pin, back_motor_backward_pin,):
        self._front_motor = ThrottleMotor(Motor(front_motor_forward_pin, front_motor_backward_pin))
        self._back_motor = ThrottleMotor(Motor(back_motor_forward_pin, back_motor_backward_pin))
        self._front_servo = SteeringMotor(AngularServo(front_servo_pin, 0, -25, 25, 0.89, 1.89))
        self._back_servo = SteeringMotor(AngularServo(back_servo_pin, 0, -25, 25, 0.89, 1.89))

        self._front_motor.stop()
        self._back_motor.stop()
        self._front_servo.set_to_inital_angle()
        self._back_servo.set_to_inital_angle()

    def stop_car(self):
        self._front_motor.stop()
        self._back_motor.stop()

    def set_car_initial_direction(self):
        self._front_servo.set_to_inital_angle()

    def set_car_direction(self, direction):
        self._front_servo.set_angle(direction)

    def run_forward(self):
        self._front_motor.run_forward()
        self._back_motor.run_forward()

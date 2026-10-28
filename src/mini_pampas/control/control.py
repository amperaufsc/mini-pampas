from gpiozero import AngularServo, Motor
from steering_motor import SteeringMotor
from throttle_motor import ThrottleMotor

class Control:
    def __init__(self, front_servo_pin, back_servo_pin, front_motor_pin, back_motor_pin):

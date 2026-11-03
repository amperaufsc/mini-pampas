from gpiozero import AngularServo, Motor
from mini_pampas.control.steering_motor import SteeringMotor
from mini_pampas.control.throttle_motor import ThrottleMotor

class Control:
    def __init__(self, front_servo_pin, back_servo_pin, front_motor_forward_pin, front_motor_backward_pin, back_motor_forward_pin, back_motor_backward_pin,):
        self._front_motor = ThrottleMotor(Motor(front_motor_forward_pin, front_motor_backward_pin))
        self._back_motor = ThrottleMotor(Motor(back_motor_forward_pin, back_motor_backward_pin))

        self._front_servo = SteeringMotor(AngularServo(front_servo_pin, min_angle=-25, max_angle=25, min_pulse_width=(0.89/1000), max_pulse_width=(1.89/1000)))
        self._back_servo = SteeringMotor(AngularServo(back_servo_pin, min_angle=-25, max_angle=25, min_pulse_width=(0.89/1000), max_pulse_width=(1.89/1000)))

    def stop_car(self):
        self._front_motor.stop()
        self._back_motor.stop()

    def set_car_initial_direction(self):
        self._front_servo.set_to_initial_angle()
        self._back_servo.set_to_initial_angle()

    def set_car_direction(self, direction):
        self._front_servo.set_angle(direction)
        self._back_servo.set_to_initial_angle()

    def run_forward(self):
        self._front_motor.run_forward()
        self._back_motor.run_forward()

    def close(self):
        self.set_car_initial_direction()
        self.stop_car()
        self._front_servo.close()
        self._back_servo.close()
        self._front_motor.close()
        self._back_motor.close()

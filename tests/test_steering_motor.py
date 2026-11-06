import pytest
from mini_pampas.control.steering_motor import SteeringMotor
from gpiozero import AngularServo

SERVO_MIN_ANGLE = -25
SERVO_MAX_ANGLE = 25
SERVO_MIN_PULSE_WIDTH = (0.89/1000)
SERVO_MAX_PULSE_WIDTH = (1.89/1000)

def make_servo():
    return AngularServo(17, min_angle=SERVO_MIN_ANGLE, max_angle=SERVO_MAX_ANGLE, min_pulse_width=SERVO_MIN_PULSE_WIDTH, max_pulse_width=SERVO_MAX_PULSE_WIDTH)

def test_start_intial_angle():
    """ Tests if the servo goes to the mid angle when started """
    servo = make_servo()
    SteeringMotor(servo)
    assert servo.angle == pytest.approx(0)

def test_set_angle_10():
    """ Tests if the servo goes to a given angle (10º) """
    servo = make_servo()
    steering = SteeringMotor(servo)

    steering.set_angle(10)

    assert servo.angle == pytest.approx(10)

def test_set_angle_n20():
    """ Tests if the servo goes to a given angle (-20º) """
    servo = make_servo()
    steering = SteeringMotor(servo)

    steering.set_angle(-20)

    assert servo.angle == pytest.approx(-20)

def test_set_angle_max_clamp():
    """ Tests if the clamp of the max angle works """
    servo = make_servo()
    steering = SteeringMotor(servo)

    steering.set_angle(100)

    assert servo.angle == pytest.approx(25)

def test_set_angle_min_clamp():
    """ Tests if the clamp of the min angle works """
    servo = make_servo()
    steering = SteeringMotor(servo)

    steering.set_angle(-100)

    assert servo.angle == pytest.approx(-25)

def test_set_to_initial():
    """ Tests if the servo goes to the mid angle """
    servo = make_servo()
    steering = SteeringMotor(servo)

    steering.set_to_initial_angle()

    assert servo.angle == pytest.approx(0)

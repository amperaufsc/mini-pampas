import pytest
from mini_pampas.control.steering_motor import SteeringMotor
from gpiozero import AngularServo

def make_servo():
    return AngularServo(17, min_angle=-25, max_angle=25, min_pulse_width=(0.89/1000), max_pulse_width=(1.89/1000))

def test_start_intial_angle():
    servo = make_servo()
    SteeringMotor(servo)
    assert servo.angle == pytest.approx(0)

def test_set_angle_10():
    servo = make_servo()
    steering = SteeringMotor(servo)

    steering.set_angle(10)

    assert servo.angle == pytest.approx(10)

def test_set_angle_n20():
    servo = make_servo()
    steering = SteeringMotor(servo)

    steering.set_angle(-20)

    assert servo.angle == pytest.approx(-20)

def test_set_angle_max_clamp():
    servo = make_servo()
    steering = SteeringMotor(servo)

    steering.set_angle(100)

    assert servo.angle == pytest.approx(25)

def test_set_angle_min_clamp():
    servo = make_servo()
    steering = SteeringMotor(servo)

    steering.set_angle(-100)

    assert servo.angle == pytest.approx(-25)

def test_set_to_initial():
    servo = make_servo()
    steering = SteeringMotor(servo)

    steering.set_to_initial_angle()

    assert servo.angle == pytest.approx(0)

import pytest
from mini_pampas.control.control import Control

PINS = dict(
    front_servo_pin=17,
    back_servo_pin=18,
    front_motor_forward_pin=19,
    front_motor_backward_pin=20,
    back_motor_forward_pin=21,
    back_motor_backward_pin=22,
)

STANDARD_THROTTLE_SPEED = 0.4729

def make_control():
    return Control(**PINS)

def test_set_car_direction_10():
    """ Tests if the front servo goes to the seted direction (10º) """
    control = make_control()
    control.set_car_direction(10)

    assert control._front_servo._servo.angle == pytest.approx(10)
    assert control._back_servo._servo.angle == pytest.approx(0)

def test_set_car_direction_n20():
    """ Tests if the front servo goes to the seted direction (-20º) """
    control = make_control()
    control.set_car_direction(-20)

    assert control._front_servo._servo.angle == pytest.approx(-20)
    assert control._back_servo._servo.angle == pytest.approx(0)

def test_set_car_initial_direction():
    """ Tests if the front servo goes to the mid direction """
    control = make_control()
    control.set_car_initial_direction()

    assert control._front_servo._servo.angle == pytest.approx(0)
    assert control._back_servo._servo.angle == pytest.approx(0)

def test_run_car():
    """ Test if the two motor goes to the standard speed """
    control = make_control()
    control.run_forward()

    assert control._front_motor._motor.value == pytest.approx(STANDARD_THROTTLE_SPEED)
    assert control._back_motor._motor.value == pytest.approx(STANDARD_THROTTLE_SPEED)
    assert control._front_motor._motor.is_active
    assert control._back_motor._motor.is_active

def test_stop_car():
    """ Tests if the two motors stops """
    control = make_control()
    control.run_forward()
    control.stop_car()

    assert not control._front_motor._motor.is_active
    assert not control._back_motor._motor.is_active

import pytest
from mini_pampas.control.throttle_motor import ThrottleMotor
from gpiozero import Motor

def make_motor():
    return Motor(17,18)

def test_start_stoped():
    motor = make_motor()
    throttle = Motor(motor)

    assert motor.value == pytest.approx(0)
    assert not motor.is_active

def test_run_forward():
    motor = make_motor()
    throttle = Motor(motor)

    throttle.run_forward()

    assert motor.is_active
    assert motor.value == pytest.approx(0.4729)
    assert motor.forward_device.value == pytest.approx(0.4729)
    assert motor.backward_device.value == pytest.approx(0)

def test_stop_motor():
    motor = make_motor()
    throttle = Motor(motor)

    throttle.stop()

    assert not motor.is_active
    assert motor.value == pytest.approx(0)

def test_run_and_stop():
    motor = make_motor()
    throttle = motor(motor)

    throttle.run_forward()

    assert motor.is_active
    assert motor.value == pytest.approx(0.4729)

    throttle.stop()

    assert not motor.is_active
    assert motor.value == pytest.approx(0)

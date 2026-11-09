import pytest
from mini_pampas.control.throttle_motor import ThrottleMotor
from gpiozero import Motor

def make_motor():
    return Motor(17,18)

def test_start_stoped():
    """ Tests if the motor starts stoped """
    motor = make_motor()
    throttle = ThrottleMotor(motor)

    assert motor.value == pytest.approx(0)
    assert not motor.is_active

def test_run_forward():
    """ Tests if the motor runs forward with the standard speed """
    motor = make_motor()
    throttle = ThrottleMotor(motor)

    throttle.run_forward()

    assert motor.is_active
    assert motor.value == pytest.approx(0.4729)
    assert motor.forward_device.value == pytest.approx(0.4729)
    assert motor.backward_device.value == pytest.approx(0)


def test_run_forward_8():
    """ Tests if the motor runs forward with a given speed (0.8) """
    motor = make_motor()
    throttle = ThrottleMotor(motor)

    throttle.run_forward(0.8)

    assert motor.is_active
    assert motor.value == pytest.approx(0.8)
    assert motor.forward_device.value == pytest.approx(0.8)
    assert motor.backward_device.value == pytest.approx(0)

def test_stop_motor():
    """ Tests if the motor stops to run """
    motor = make_motor()
    throttle = ThrottleMotor(motor)

    throttle.run_forward()
    throttle.stop()

    assert not motor.is_active
    assert motor.value == pytest.approx(0)

def test_run_and_stop():
    """ Tests the run and stop commands together """
    motor = make_motor()
    throttle = ThrottleMotor(motor)

    throttle.run_forward()

    assert motor.is_active
    assert motor.value == pytest.approx(0.4729)

    throttle.stop()

    assert not motor.is_active
    assert motor.value == pytest.approx(0)

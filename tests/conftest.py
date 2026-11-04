import pytest
from gpiozero import Device
from gpiozero.pins.mock import MockFactory, MockPWMPin


@pytest.fixture(autouse=True)
def gpiozero_mock():
    """Instala pinos falsos antes de cada teste e limpa depois."""
    saved = Device.pin_factory
    Device.pin_factory = MockFactory(pin_class=MockPWMPin)
    yield
    Device.pin_factory.reset()
    Device.pin_factory = saved

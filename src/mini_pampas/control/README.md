## Steering and Throttle

This project aims to develop the control system for the steering and throttle motors for *mini-pampas project*

## Throttle control

For the throttle control, I decided to run the motor at a constant speed to keep the project simple at first.

For motor control, we are using a BTS7960 motor driver module, which has two PWM input pins: one for forward speed and the other for reverse speed. I will use only the forward pin and set the **PWM signal** to a fixed duty cycle so that the motor output voltage stays around 7 V, which is about half of our battery voltage. For the 7 V, i'll use 120 of dutycycle.

![](https://external-content.duckduckgo.com/iu/?u=https%3A%2F%2Fdeepbluembedded.com%2Fwp-content%2Fuploads%2F2024%2F01%2FTBS7960-Pinout.jpg&f=1&nofb=1&ipt=2f788744f992d0a39ce3a0185086254dfd2bd33520fb12ddb0d9f396ea430991&ipo=images)

## Steering control

For the steering control, we are using a servo with 1000-2000 us range, centered at 1500 us. For the moment, I don't know the servo limit angles, so I'll keep the max angles in -60º and 60º.

At first I'll just send directly the **PWM signal** with dutycycle that matches the angle setpoint given by planning project.

## gpiozero

For the control of both steering and throttle motor, I'll be using **gpiozero library**. The gpiozero comes with classes for Motors ans Servos, with functions to control its angle or speed sending PWM signal with the needed dutycycle.

### Factories, Mock Factory and pytest

The `factories` in gpiozero library is the way that the software talks with the hardware to do the control of the pin. In this project I'm gonna use the `Mock Facotry`. This factory creates a fake pin, allowing the user to realize tests of the pins and Output Devices without any real hardware.

The pytest library is a framework that makes the tests of codes easier. Using the Mock Factory and pytest, I'm gonna write tests for each one of the classes in control project (Control, Throttle and Steering) and verify if all methods of its classes are working the way i expect.

### conftest.py

The conftest.py is the file that configure what is necessary to run the tests, here that I'll define to always create pins and Output Devices as Mock Factories when running tests with pytest.

### Smoothing steering control

I'll use some techniques to smooth the lateral control of mini-pampas:

- `Exponential moving average`: makes the servo rotate to just a fraction of the desired angle.
```python
tau = 1.0 / (2.0 * math.pi * self.fc)
alpha = dt / (tau + dt)
self.filtered += alpha * (target - self.filtered)
```

- `Rate limiter`: limits the angular rate of the servo motor, limiting how fast it comes to the desired angle.
```python
max_step = self.v_max * dt  # degrees per cycle
error = self.filtered - self.output
self.output += max(-max_step, min(max_step, error))
```

With these two methods, the servo will make smooth turns to the desired angles.

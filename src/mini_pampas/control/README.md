## Steering and Throttle

This project aims to develop the **control system** for the *steering and throttle motors* for *mini-pampas project*

## Steering control

For the **steering control**, we are using a servo with 1000-2000 us range, centered at 1500 us. For the moment, I don't know the servo limit angles, so I'll keep the max angles in -60º and 60º.

At first I'll just send directly the *PWM signal* with dutycycle that matches the angle setpoint given by *planning project*.

## pigpio library

For the control of the **PWM signals** for both steering and throttle motor, I'll be using the **pigpio library**. The *pigpio* library allows the control of the *GPIO* pins of the *Raspoberry Pi*, including hardware timed *PWM control*.

The **gpipio** library has the *pigpio.pi* class, that comes with some functions to control the *gpio*, like *write*, *set_mode*, *set_PWM_dutycycle* or *set_servo_pulsewidth*.

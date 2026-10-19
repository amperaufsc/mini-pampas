## Steering and Throttle

This project aims to develop the **control system** for the *steering and throttle motors* for *mini-pampas project*

## Throttle control

For the **throttle control**, I decided to run the motor at a constant speed to keep the project simple at first.

For motor control, we are using a *BTS7960* motor driver module, which has two *PWM input pins*: one for forward speed and the other for reverse speed. I will use only the forward pin and set the **PWM signal** to a fixed duty cycle so that the motor output voltage stays around 7 V, which is about half of our battery voltage.

## pigpio library

For the control of the **PWM signals** for both steering and throttle motor, I'll be using the **pigpio library**. The *pigpio* library allows the control of the *GPIO* pins of the *Raspoberry Pi*, including hardware timed *PWM control*.

The **gpipio** library has the *pigpio.pi* class, that comes with some functions to control the *gpio*, like *write*, *set_mode*, *set_PWM_dutycycle*. The main fucntion here will be *set_PWM_dutycycle*, that i'll use to send the fixed *PWM signal* to the pin of motor driver.

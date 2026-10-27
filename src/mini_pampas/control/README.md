## Steering and Throttle

This project aims to develop the **control system** for the steering and throttle motors for *mini-pampas project*

## Throttle control

For the **throttle control**, I decided to run the motor at a constant speed to keep the project simple at first.

For motor control, we are using a BTS7960 motor driver module, which has two PWM input pins: one for forward speed and the other for reverse speed. I will use only the forward pin and set the **PWM signal** to a fixed duty cycle so that the motor output voltage stays around 7 V, which is about half of our battery voltage. For the 7 V, i'll use 120 of dutycycle.

## Steering control

For the **steering control**, we are using a servo with 1000-2000 us range, centered at 1500 us. For the moment, I don't know the servo limit angles, so I'll keep the max angles in -60º and 60º.

At first I'll just send directly the **PWM signal** with dutycycle that matches the angle setpoint given by planning project.

## gpiozero

For the control of both steering and throttle motor, I'll be using **gpiozero library**. The gpiozero comes with classes for Motors ans Servos, with functions to control its angle or speed sending PWM signal with the needed dutycycle.

For the throttle motor I'm gonna use the `Motor` class. I'll be using just one pin for the motor, because the mini-pampas will only move forward. The Motor class has functions to control the speed of the motor, I'll be using just the `forward` function to drive mini-pampas.
- `forward(speed=1)`: Drive the motor forward. The speed parameter is a float between 0.0 and 1.0 to control the speed of the motor, 0.0 stopping the motor and 1.0 the max speed. I'll turn on the motor at speed 0.4729 to keep the voltage at 7 V.

For the steering motor I'm gonna use the `AngularServo`class. Its constructor requests the parameters:
- `pin`: the pin where the servo motor is connected
- `inital_angle`: the initial angle of the servo motor
- `min_angle`: the minimum angle of the servo motor
- `max_angle`: the maximum angle of the servo motor
- `min_pulse_width`: the minimum pulse width of the servo motor
- `max_pulse_width`: the maximum pulse width of the servo motor
To rotate the servo motor, I'll be controlling the angle property of the steering motor.
- `angle`: the position of the servo as an angle measured in degrees.

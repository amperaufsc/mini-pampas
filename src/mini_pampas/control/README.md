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

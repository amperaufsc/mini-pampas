## Steering and Throttle

This project aims to develop the **control system** for the steering and throttle motors for *mini-pampas project*

## Throttle control

For the **throttle control**, I decided to run the motor at a constant speed to keep the project simple at first.

For motor control, we are using a BTS7960 motor driver module, which has two PWM input pins: one for forward speed and the other for reverse speed. I will use only the forward pin and set the **PWM signal** to a fixed duty cycle so that the motor output voltage stays around 7 V, which is about half of our battery voltage. For the 7 V, i'll use 120 of dutycycle.

## Steering control

For the **steering control**, we are using a servo with 1000-2000 us range, centered at 1500 us. For the moment, I don't know the servo limit angles, so I'll keep the max angles in -60º and 60º.

At first I'll just send directly the **PWM signal** with dutycycle that matches the angle setpoint given by planning project.

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

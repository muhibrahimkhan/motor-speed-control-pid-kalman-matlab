# Motor Speed Control with PID and Kalman Filtering — MATLAB

A MATLAB simulation of a closed-loop DC motor speed controller using a PID controller and Kalman filter for state estimation under noisy sensor conditions.

## What This Does

Simulates a DC motor being driven to track a changing speed reference. The motor's speed sensor produces noisy measurements, so a Kalman filter runs in the loop to estimate the true speed before feeding it back to the PID controller. The controller handles multiple step changes in target speed across an 8-second simulation.

## Why I Built It

I wanted to apply what I learned in my control systems coursework — PID tuning, state-space modeling, and Kalman filtering — in a single integrated simulation that resembles how a real embedded control loop works. The project helped me connect theory (frequency response, stability, estimation) to a concrete implementation.

## How It Works

- DC motor modeled as a discrete-time first-order system in state-space form
- Gaussian noise added to sensor output to simulate real measurement conditions
- Kalman filter runs predict/update steps each timestep to estimate true motor speed
- PID controller uses the filtered estimate as feedback, with anti-windup on the integral term to prevent saturation drift
- Performance evaluated automatically: rise time, overshoot, settling time, and steady-state error

## How to Run

Open MATLAB in the project folder and run:

```matlab
results = motor_pid_kalman_demo();
```

Plots and performance metrics are generated automatically.

## Skills Demonstrated

PID design and tuning, Kalman filtering, discrete-time state-space modeling, sensor noise handling, closed-loop control simulation, MATLAB

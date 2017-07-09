# CarND-Controls-PID

[//]: # (Image References)

[overview]: ./overview.gif "Overview"

Overview
---

This project implements a PID controller in C++ to maneuver the vehicle around the simulation track. The cross track error (CTE) and the velocity (mph) are provided by the simulator in order to compute the appropriate steering angle as the output of the controller. 

![alt text][overview]

Reflection
---

### PID Controller and effect of each component

A proportional–integral–derivative controller (PID controller) is a control loop feedback mechanism widely used in industrial control to calculate an error value E(n) as the difference between a desired set point and a measured process variable and applies a correction based on proportional, integral, and derivative terms. In the context of this project, a vehicle simulator produces the error signal as the distance between actual position of the car on the road and a reference trajectory called cross-track error (cte). The PID controller output is the steering angle such that it minimises the distance to this reference trajectory. 

```
U(n) = - Kp * E(n) - Ki * Ei(n) - Kd * Ed(n)
```

where
* `E(n)` - cross-track error which is the error between the currently measured value and setpoint
* `Ei(n)` - sum of the instantaneous error over time till current moment
* `Ed(n)` - time derivative of error
* `Kp, Ki, Kd` - proportional, integral and derivative gains

#### P - proportional

`Kp * E(n)` is the proportional term that produce correction signal that is proportional to the cross-track error. If proportional gain `Kp` is too high, the system can become unstable whereas, a small gain results in a small corrections to a large errors. 

#### I - integral

`Ki * Ei(n)` is the integral term that accelerates the movement of the process towards setpoint and eliminates the residual steady-state error that occurs with a pure proportional term. Since the integral term sums up the cross-track error over time, it is useful in high speeds to sum a large error signal quickly. Integral term also mitigates any biases such as if a zero steering angle does not correspond to a straight trajectory.

#### D - differential

`Kd * Ed(n)` is the derivative term that predicts error behavior and mitigates the system from deviating from setpoint in the future. However, if `Kd` is too large, the controller can become less sensitive. 

### Hyper-parameter Tuning

There are several known methods of tuning a PID controller. A more systematic and algorithmic approach such as Twiddle algorithm, accompanied by code to automatically restart the simulator can be done. Ziegler-Nichols method of tuning is another popular method of tuning PID controllers. However, I decided to tune the hyper-parameters manually as it provides a more intuitive understanding of the importance of different contributions of each gains. 

The steps I took were as follows:
* Set reasonable `Kp` value while Ki` and `Kd` are zero
* Increase `Kd` until oscillations decrease substantially
* If car goes out of track and if car is slow to react, increase `Kp` or `Ki`. 
* If car goes out of track and if it oscillates a lot, decrease `Kp` or `Ki`
* Repeat

After a lot of trials, I settled with the following hyper-parameters:
* Kp = 0.2
* Ki = 0.001
* Kd = 2.2

### Simulation Video

With the tuned PID parameters, the car drives smoothly within the lane for many laps safely. Here is my implementation video:

[![Simulation Video](https://img.youtube.com/vi/PHBFTpUYXFY/0.jpg)](https://www.youtube.com/watch?v=PHBFTpUYXFY)

### Future Improvements

After going through a lot of manual tweaking of the gains, I would be more inclined to using a more systematic approach such as Twiddle algorithm in the future to speed up the process.

Run Instructions
---

### Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

### Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

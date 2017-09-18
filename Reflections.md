### Self Driving Car Engineering Nanodegree

---

The goals / steps of this project are the following:

* Compilation: Code must compile without errors with cmake and make.
* Implementation:
* Student describes their model in detail. This includes the state, actuators and update equations.
* Student discusses the reasoning behind the chosen N (timestep length) and dt (elapsed duration between timesteps) values. Additionally the student details the previous values tried.
* A polynomial is fitted to waypoints. If the student preprocesses waypoints, the vehicle state, and/or actuators prior to the MPC procedure it is described.
* The student implements Model Predictive Control that handles a 100 millisecond latency. Student provides details on how they deal with latency.
* Simulation: No tire may leave the drivable portion of the track surface. The car may not pop up onto ledges or roll over any surfaces that would otherwise be considered unsafe. The car can not go over the curb, but, driving on the lines before the curb is ok.

[//]: # (Image References)
[image1]: ./output/img_pid.png

---
## [Rubric](https://review.udacity.com/#!/rubrics/896/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.

---
### Writeup / README

#### [Here](https://github.com/abhardwajnv/CarND-MPC-Project/blob/master/Reflections.md) is the writeup for this project.

You are reading it!

---

## Implementation

### 1. Student describes their model in detail. This includes the state, actuators and update equations.

I used Kinematic model as an implementation for this Vehicle Model. Kinematic models are simplifications of dynamic models which ignores the dynamic forces.
Though this simplification reduces the accuracy of the model, but it makes them more tracable and non-linear.

This part of Code was implemented in MPC.cpp under class FG_eval.

This model has 6 different state values & 2 actuators which are as follows:

#### State Values:

* px --> Position in X
* py --> Position in Y
* psi --> Car Direction of Heading
* cte --> Cross Track Error
* epsi --> PSI Error
* v --> Velocity

#### Actuators

* steering angle (delta)
* throttle (a)

Following are the Model update equations used:

* x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
* y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
* psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
* v_[t+1] = v[t] + a[t] * dt
* cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
* epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt

### 2. Student discusses the reasoning behind the chosen N (timestep length) and dt (elapsed duration between timesteps) values. Additionally the student details the previous values tried.

The values N (timestep length) & dt (elapsed duration between timesteps) were used the same as mentioned in the Model Predictive Control chapter.
Idea is to decrease the cross track error to 0 by altering timestep length and the duration between timesteps.

Here, higher the value of timestep length the more slower the controller will process since it has to optimize the control inputs, Ipopt & solver etc which in turn will cause the simulation to be slower as well. So i tried to keep N as optimal as possible.
In addition dt we would not like too small because the smaller the duration between timestep the more steps controller will run which makes it again more computationally difficult, so again we have to be cautious while keeping a good enough small dt value.
Hence i start with the chapter given values of 25 & 0.05 each which worked perfectly for my submission.

Although i tried changing the values of N (10, 15, 20, 25, 30) & dt (0.1, 0.01, 0.05, 0.001) for experimentation but found better results with the current values hence decided to keep them itself.
I observed with low values of N & dt the simulation was slower getting deviated at the dirt patch turn.
Same thing i observed when i tried increasing the ref_v (30, 40, 50, 60, 100) so i decided to finalize the ref_v at 40.

Below is the code snippet used in MPC.cpp.

          size_t N = 25 ;
          double dt = 0.05 ;
	
### 3. A polynomial is fitted to waypoints. If the student preprocesses waypoints, the vehicle state, and/or actuators prior to the MPC procedure it is described.

* MPC Preprocessing

The waypoints of Car reference trajectory are given in the MAP co-ordinate system. These waypoints are transformed into the vehicle co-ordinate system.
This transformation is required to perform the calculations consistently in vehicle co-ordinate system.
Below is the code snippet from main.cpp

          for (unsigned int i = 0; i < ptsx.size(); i++)
          {
            double shift_x = ptsx[i] - px;
            double shift_y = ptsy[i] - py;
            ptsx[i] = shift_x * cos(0-psi) - shift_y * sin(0-psi);
            ptsy[i] = shift_x * sin(0-psi) + shift_y * cos(0-psi);
          }

* Polynomial Fitting

As the waypoints are in vehicle co-ordinate system, a third order polynomial is then fitted.
Below is the code snippet from main.cpp

	  Eigen::VectorXd x_temp = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(ptsx.data(), ptsx.size());
          Eigen::VectorXd y_temp = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(ptsy.data(), ptsy.size());

          // Measure Polynomial Coefficients.
          auto coeffs = polyfit(x_temp, y_temp, 3);

Complete code for this step can be found in main.cpp


### 4. The student implements Model Predictive Control that handles a 100 millisecond latency. Student provides details on how they deal with latency.

Here 100 miliseconds of latency is introduced to the actuators. Since this is an additional latency added we had to incorporate it otherwise trajectory of path would have been compromised.
Also the car movement will be affected due to the osscilations caused. 
Hence to tackle this issue i predicted the state forward at the latency time itself before giving that input to the solver.

Steps used:

* First calculate initial latency time for state values (px, py & psi).
* Then calculate the errors (cte & epsi) at the beginning of timestep.
* Now update the error values to incorporate latency considerations.
* Finally feed the state values and coeffs to the solver.

Following is the code snippet from main.cpp showing the latency consideration.

	  double steer_value;
          double throttle_value;

          v *= 1.609344      // Convert Speed
          steer_value = j[1]["steering_angle"];
          throttle_value = j[1]["throttle"];
          const double Lf = 2.67;
          
          // Predict the state
          const double latency = 0.1;
          px += v * cos(psi) * latency;
          py += v * sin(psi) * latency;
          psi += - v * steer_value/Lf * latency;
          v += throttle_value * latency;
		  
	  // Calculate cte & epsi
          double cte = polyeval(coeffs, 0);
          double epsi = psi - atan(coeffs[1]);

          // Incorporate latency considerations
          cte += v * sin(epsi) * latency;
          epsi += v * steer_value/Lf * latency;

          // Define the state vector
          Eigen::VectorXd state(6);
          state << 0, 0, 0, v, cte, epsi;

          // Call MPC Solver
          auto vars = mpc.Solve(state, coeffs);
		  
Complete Code for this step can be found in main.cpp under main function.

## References:

* https://en.wikipedia.org/wiki/Model_predictive_control
* https://www.youtube.com/watch?v=bOQuhpz3YfU&list=PLAwxTw4SYaPnfR7TzRZN-uxlxGbqxhtm2&index=5

# Extended Kalman Filter 
Self-Driving Car Engineer Nanodegree Program

In this project I use a Kalman Filter to estimate the state of a moving object of interest with noisy lidar and redar measurements. To measure the accuracy of the positions and velocity predicted by the Kalman Filter, RMSE values between predictions and the ground truth included in the data is displayed. Final RMSE values are lower than the tolerance outlined in the [project rubric](https://review.udacity.com/#!/rubrics/748/view).

<figure>
	<img src="./final_screenshot.png" width="40%" height="40%" />
	<figcaption>RMSE values for [x, y, vx, vy] < [0.11, 0.11, 0.52, 0.52]</figcaption>
</figure>

## Code description

A starter code is given by the Udacity project contained in `/src`. For the Kalman Filter to be fully implemented the following files where completed:

- `tools.cpp`: funtions to calculate root mean squared error (RMSE) and the Jacobian matrix
- `FusionEKF.cpp`: initializes the Kalman Filter on first data point, prepare the Q and F matrices, calls the prediction step, and depending on the data source calls the radar or lidar update functions
- `kalman_filter.cpp`: contains the Predict(), Update(), and UpdateEKF() functions



## Installation and Build

This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases).

Once you clone this repo, it includes two files that can be used to set up and install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. For windows you can use either Docker, VMware, or even [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) to install uWebSocketIO. 

Once the install for uWebSocketIO is complete, the main program can be built and run by doing the following from the project top directory.

- Make a build directory: `mkdir build && cd build`
- Compile: `cmake .. && make` 
   * On windows, you may need to run: `cmake .. -G "Unix Makefiles" && make`
- Run it: `./ExtendedKF `

Refer to the Udacity project repository for more detail intalation instructions.

## Other Important Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)


# Extended Kalman Filter Project
Self-Driving Car Engineer Nanodegree Program

# Overview
This project consists of implementing an [Extended Kalman Filter](https://en.wikipedia.org/wiki/Extended_Kalman_filter) with C++. A simulator provided by Udacity ([it could be downloaded here](https://github.com/udacity/self-driving-car-sim/releases)) generates noisy RADAR and LIDAR measurements of the position and velocity of an object, and the Extended Kalman Filter[EKF] must fusion those measurements to predict the position of the object. The communication between the simulator and the EKF is done using [WebSocket](https://en.wikipedia.org/wiki/WebSocket) using the [uWebSockets](https://github.com/uNetworking/uWebSockets) implementation on the EKF side.
To get this project started, Udacity provides a seed project that could be found (here)(https://github.com/udacity/CarND-Extended-Kalman-Filter-Project).

# Prerequisites

The project has the following dependencies (from Udacity's seed project):

- cmake >= 3.5
- make >= 4.1
- gcc/g++ >= 5.4
- Udacity's simulator.

I used virtualbox to spin up a ubuntu machine and ran the code in the virtual machine

I ran the sim on local mac. Connection between sim and vistual machine is achieved by 
exposing port via following line in the Vagrantfile. Look inside ud_vagrant folder.

```config.vm.network "forwarded_port", guest: 4567, host: 4567```

Once virtual machine is up it will clone git repo and run ./install-ubuntu.sh

# Compiling and executing the project

These are the suggested steps:

- Clone the repo and cd to it on a Terminal.
- Create the build directory: `mkdir build`
- `cd build`
- `cmake ..`
- `make`: This will create two executables
  - `ExtendedKF` : EKF implementation.
  - `Tests` : Unit Tests using [Catch](https://github.com/philsquared/Catch/blob/master/docs/tutorial.md).

## Running the tests

In the build directory, execute `./Tests`. Output:

```
Invalid estimation or ground_truth data
Invalid estimation or ground_truth data
Invalid estimation or ground_truth data
CalculateJacobian () - Error - not enough state
CalculateJacobian () - Error - Division by Zero
Rho too small - Prevent Error - Division by Zero
===============================================================================
All tests passed (27 assertions in 4 test cases)
```

## Running the Filter/Simulator

Start sim on mac osx
Run the filter inside the virtual machine by executing ./ExtendedKF inside the build folder

```
/ExtendedKF
Listening to port 4567
Connected!!!
```

Connection successful.

The simulator provides two datasets. The difference between them are:

- The direction the car (the object) is moving.
- The order the first measurement is sent to the EKF. On dataset 1, the LIDAR measurement is sent first. On the dataset 2, the RADAR measurement is sent first.


Simulator Output with dataset1

[![Simulator Dataset1 Run](https://img.youtube.com/vi/HORefcB8X-0/0.jpg)](https://www.youtube.com/watch?v=HORefcB8X-0)

Simulator Output with dataset2

[![Simulator Dataset2 Run](https://img.youtube.com/vi/YM5zFhSGfm4/0.jpg)](https://www.youtube.com/watch?v=YM5zFhSGfm4)

## Compiling

### Your code should compile

The code compiles without errors. I did change the [CMackeLists.txt](./CMakeLists.txt) to add the creation of the `./Tests`. Works on Ubuntu 16.04

## Accuracy

### px, py, vx, vy output coordinates must have an RMSE <= [.11, .11, 0.52, 0.52] when using the file: "obj_pose-laser-radar-synthetic-input.txt which is the same data file the simulator uses for Dataset 1"

Accuracy of EKF:

- Dataset 1: RMSE <= [0.0973178, 0.0854597, 0.451267, 0.439935]
- Dataset 2: RMSE <= [0.0725678, 0.0964738, 0.421634, 0.493199]

## Following the Correct Algorithm

### Your Sensor Fusion algorithm follows the general processing flow as taught in the preceding lessons.

The Kalman filter implementation can be found [src/kalman_filter.cpp](./src/kalman_filter.cpp) and it is used to predict at [src/FusionEKF.cpp](./src/FusionEKF.cpp#L148) line 148 and to update line 159 to 170.

### Your Kalman Filter algorithm handles the first measurements appropriately.

The first measurement is handled at [src/FusionEKF.cpp](./src/FusionEKF.cpp#L62) from line 62 to line 108.

### Your Kalman Filter algorithm first predicts then updates.

The predict operation is at [src/FusionEKF.cpp](./src/FusionEKF.cpp#L148) line 148 and to update operation is from line 159 to 170.

### Your Kalman Filter can handle radar and lidar measurements.

Different type of measurements are handled in two places in [src/FusionEKF.cpp](./src/FusionEKF.cpp):

- For the first measurement from line 62 to line 108.
- For the predict part at line 148.
- For the update part from line 159 to 170.

## Code Efficiency

### Your algorithm should avoid unnecessary calculations.

An example of this calculation optimization is when the Q matrix is calculated [src/kalman_filter.cpp](./src/kalman_filter.cpp#L41) line 41 and line 63.

Made utility functions for polar to cartesian and vice versa in [src/tools.cpp](./src/tools.cpp#L81) lines 81 and 102








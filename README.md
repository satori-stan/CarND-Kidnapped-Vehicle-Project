# Kidnapped Vehicle Project
Self-Driving Car Engineer Nanodegree Program

In this project I implement a 2 dimensional particle filter in C++. The particle filter will be given a map and some initial localization information (analogous to what a GPS would provide). At each time step the filter will also get observation and control data.

The required localization follows a "robot" that has been kidnapped and transported to a new location. Luckily it has a map of this location, a (noisy) GPS estimate of its initial location, and lots of (noisy) sensor and control data.

## Objective

The goal of this project is to accurately localize a moving object using a particle filter, first by getting a broad idea of its location and then matching sensor observations with a map of landmarks in the area.

A successful implementation requires:

* Compilation on any platform (Linux, Mac, Windows).
* Localize the vehicle within the desired accuracy (as verified by the simulator).
* The localization must happen fast enough to be used in a moving object (a car). In the simulator, the particle filter must accurately localize the car for all time steps in under 100 seconds.
* Following the logic of a particle filter (initialization, prediction, particle weight update, resampling).

[image1]: ./successful_run.png
### Here I will consider each point individually and explain how I addressed them in my implementation.

#### 1. Compilation on any platform

The starter code provided for the project was already meant to compile correctly in Linux, Mac and Windows; although compilation in Windows required installing Windows 10 Bash on Ubuntu or using Docker to create a virtual environment.

Since my current platform is Windows but I have low resources, I decided to go for a native approach to avoid the installation of such resource-intensive environments. This decision was originally taken during the implementation of the first project of the term [Extended-KalmanFilter](https://github.com/satori-stan/CarND-Extended-Kalman-Filter-Project). Then, it had a non-trivial time cost, which I was able to capitalize on for this project. There is little in the repository that will give my choice away, except for a couple of lines at the end of the CMakeLists.txt file where the names of the libraries are redefined when a Windows environment is found.

The main program file (main.cpp) was modified slightly to accommodate api changes in uWS 0.14.4. This file receives a telemetry message from the simulator and passes it to the ParticleFilter class, where the information is processed.

#### 2. Localize the vehicle within the desired accuracy

While the precise accuracy targets are not specified, I did fail the accuracy test for the first few iterations of my code so I know what to look for. In its current incarnation, the program will run without going over the maximum accuracy error limits for either the position or yaw.

Since the localization is done with a Particle Filter, accuracy is measured by calculating the average weighted error of the particles over all the timesteps of the test. The graphical representation in the simulator is done using the best particle of the filter (the one with the highest calculated weight, which is a measure of the probability that the particle represents the tracked object).

For a number of recorded runs, the X position error was in average around 0.140, Y error around 0.160 and yaw error at 0.005.

The weights are calculated in the ParticleFilter::updateWeights method (particle_filter.cpp:120). The function deals first with mapping the observations to each particle and finding the group of landmarks that will best represent the position of the particle in the map (lines 133 to 180). The last step is calculating the probability of the particle representing the tracked object (lines 182 to 212).

There are a number of lessons learned achieving this goal:

1. Failing to reset the particle ids during the prediction step will cause the filter to fail. The first few iterations of the code failed to do this and for my implementation, the index of the weights vector that got the probability is determined by the particle id (line 211). This caused that after a couple of runs, only one weight was being updated, leading to an exponential drift of the localization with respect to the ground truth.
1. A small number of particles is great for debugging, good for speed but bad for localization. The initial number of particles used was 10. For some runs the localization would suddenly drift away from the ground truth. After increasing the number of particles to 15 (particle_filter.h:54) this error disappeared and performance didn't see much impact. A question I still have is how to know how many particles are enough? My intuition is that more particles achieve better accuracy and the limit is really set by the performance constraints of the system (i.e. how fast the result must be obtained to be of use).
1. During the last step of the calculation of the weights, in a small number of runs (~3), the weights of all the particles came back as zero. The occurrence was rare enough that I couldn't find the cause for it, but it leads me to wonder whether what to do when the best estimate is far enough from the actual location to be a cause of concern (as measured by the best probability of all the particles). My guess is that the localization estimate must be cross-checked with some other sensor's data. In the example of our self-driving car, GPS data must be periodically used to inform the particle filter. Inertial sensor data will obviously be valuable as well, since the accuracy of GPS is too low and unreliable to prevent a collision if the vehicle is miss-localized.

#### 3. Run fast

The test runs for slightly over 2K timesteps. Again, during the first few iterations (especially while debugging) the allotted time (100 seconds) would be spent and the test failed. My guess is that the run time depends on the machine running the code, but in debugging sessions (without any breakpoints) the program would finish in under 70 seconds. Running the program outside of the IDE allowed the test to be run in under 50 seconds.

![standalone test screenshot][image1]

A couple of design decisions were made that improve performance.

The first is the use of C++'s discrete_distribution class for the resampling step. In the course material, a resampling wheel is suggested as a sound strategy for randomly sampling the particles with the largest weights to survive for the next cycle. A downside of this strategy is that there is looping involved that may or may not be optimized by the compiler. The weighted discrete distribution available in the C++ standard library is already optimized. In a set of 5 runs under similar conditions, the discrete distribution outperformed the resampling wheel by 4% without negatively impacting the accuracy of the filter, so the final solution uses this method (particle_filter.cpp:221). In the code, both implementations can be found for reference (lines 224 and 252).

The second is that since the particle filter relies heavily on vectors and loops, key values used for calculations are extracted from the vector and placed in local variables in the scope. This allows the program to (depending on the hardware) keep such values closer to where the program uses them, (hopefully) reducing the number of computing cycles.

I believe it is possible to further improve the performance. A more thorough review of the code could identify unnecessary or inefficient loops. It would also be possible to use a library such as Eigen for optimized vector operations. And finally, changing the code to run on a GPU.

#### 4. Follow the logic of a particle filter

This part should be obvious by now, but the program uses a vector of particles that have a position (x and y), heading and weight. The first three values are used for the localization. The weight is only used for the process.

These particles get informed initially of a location (in our self-driving car example, by means of a GPS reading) that is thought to have low accuracy. The particles are the dispersed randomly in the probable location space around the initial reading.

A second set of readings is obtained from other, more accurate, sensor (in our example, LIDAR readings of landmarks). This second set of readings is then cross-referenced with a map of landmarks and matched to each particle to identify the ones that are most likely to represent the object we are tracking. Only those that have high likelihoods are maintained (although the number of particles remains the same, so the most likely particles are represented more than once).

With every new reading, a prediction is made as to where the object is now (this is done based on the motion model of the object) and again the readings of the landmark positions with respect to the object are cross-referenced with a map and particles with less likelihood of representing the object are discarded.

## This repository

The project uses the following:

* CMake 3.5 and Make 4.1 (3.81 for Windows)
* Zlib, OpenSSL, and Libuv.
* [uWebSocketIO](https://github.com/uWebSockets/uWebSockets)
* The Udacity Self-Driving Car Engineer Nanodegree Program Term 2 Simulator, which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

This repository includes two files that can be used to set up and intall uWebSocketIO for either Linux or Mac systems. For windows you can use either Docker, VMware, or even Windows 10 Bash on Ubuntu to install uWebSocketIO.

Once the install for uWebSocketIO is complete, the main program can be built and ran by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./particle_filter

Alternatively some scripts have been included to streamline this process, these can be leveraged by executing the following in the top directory of the project:

1. ./clean.sh
2. ./build.sh
3. ./run.sh

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

Note that the programs that need to be written to accomplish the project are src/particle_filter.cpp, and particle_filter.h

The program main.cpp has already been filled out, but feel free to modify it.

Here is the main protcol that main.cpp uses for uWebSocketIO in communicating with the simulator.

INPUT: values provided by the simulator to the c++ program

// sense noisy position data from the simulator

["sense_x"] 

["sense_y"] 

["sense_theta"] 

// get the previous velocity and yaw rate to predict the particle's transitioned state

["previous_velocity"]

["previous_yawrate"]

// receive noisy observation data from the simulator, in a respective list of x/y values

["sense_observations_x"] 

["sense_observations_y"] 


OUTPUT: values provided by the c++ program to the simulator

// best particle values used for calculating the error evaluation

["best_particle_x"]

["best_particle_y"]

["best_particle_theta"] 

//Optional message data used for debugging particle's sensing and associations

// for respective (x,y) sensed positions ID label 

["best_particle_associations"]

// for respective (x,y) sensed positions

["best_particle_sense_x"] <= list of sensed x positions

["best_particle_sense_y"] <= list of sensed y positions


Your job is to build out the methods in `particle_filter.cpp` until the simulator output says:

```
Success! Your particle filter passed!
```

### Inputs to the Particle Filter
You can find the inputs to the particle filter in the `data` directory. 

#### The Map*
`map_data.txt` includes the position of landmarks (in meters) on an arbitrary Cartesian coordinate system. Each row has three columns
1. x position
2. y position
3. landmark id



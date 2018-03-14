## PID Controller Project
---
Yangchun Luo<br>
March 13, 2018

This is the assignment for Udacity's Self-Driving Car Term 2 Project 4.

---
This project revisits the lake race track from the Behavioral Cloning Project. This time, however, we implement a PID controller in C++ to maneuver the vehicle around the track.

### To build

```bash
mkdir build
cd build
cmake ..
make
```

### To run

Download the [Term 2 simulator suite](https://github.com/udacity/self-driving-car-sim/releases). Open it and choose the Kidnapped Vehicle project.

Other setup information can be found in the original [README](README-orig.md) file.

To run the twiddling alogrithm to tune hyperparameters:

```bash
./twiddler
```

To run the PID controller:

```bash
./pid
```

### PID Components Reflection

The P component is proportional to the current crosstrek error (CTE). It is the main "force" to steer the vehicle back on track.

The D component is the temporal derivative of the CTE. It is used to count-steer the vehicle to avoid the ever-lasting oscillation created by the P component.

The I component is the accumlation of all CTEs for the entire duration. It is used to counter the system bias.

### Hyperparameter Tuning

I implemented the twiddling algorithm inside the client code of the simulator (`src/twidder.cpp`). It is complied as a standalone executable which interacts with the simulator.

The implementation has a different organization compared to what is taught in the class, because there is no easy way to package the a simulation run as a separate function to be called by the twiddling algorithm. Instead, the latter is embedded into the simulator's `onMessage` callback function with extra state variables to remember what the current run is about. Please the source code for details.

Through manually setting the initial values of `p` and `dp` and tuning them with the twiddling alogrithm, the following values have been reached which optimize the squared sum of CTE of the first 500 simulation steps/callbacks.

* Kp = 0.2
* Ki = 0.004
* Kd = 8

A portion of the simulation recording can be found in the following [file](recording.mov).

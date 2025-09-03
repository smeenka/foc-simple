# foc-simple

Library for controlling blcd motors with Field Oriented Control (foc)
The library dox c0.3.0 is used.

With this library one can control blcd motors, with embassy, with rtic.

All plumbing needed to run

## Goals

* Reuse of foc v0.3.0 library with same goals here
* Exclusively use fixed-point math for all FOC calculations, using the fixed crate.
* Generic over angle sensors, current sensors, and PWM drivers.
* No heap allocations anywhere.
* Can be used with rtic or  embassy
* Application layer is fully decoupled from the foc layer. In this way the library is easy to use. (No owning issues to solve)
* Provides a basic serial command terminal for controlling the motors
* Multiple motors can be controlled
* Able to do the startup calibration of the motor


The library does provide interfaces for:
* angle sensor
* current sensor
* serial communictation
* pwm driver

The user of this library should implement these interfaces. The user will have to solve all the nitty gritty details for controlling the hardware.

In this way the library can stay very generic. 

Note that the current control loop, with the feedback of  current sensors, is not yet implemented.

# Layers

The library does contain the following layers
* The trait definititions to be implemented by the user
* A fully syncronous layer, with does have no dependencies towards rtic or embassy.
* An async wrapping layer for embassy
* An async wrapping layer for rtic 2.0
* Examples for controlling motors, with embassy or rtic, with 2 different platforms (nucleo-f103rb and storm32). Examples are available in the github repository


## Buiding this library

To build this library feature "rtic" or "embassy" must be selected:

* cargo build --release --features="rtic"

or
*  cargo build --release --features="embassy" 
# foc-simple

Library for controlling blcd motors with Field Oriented Control (foc).
With this library one can control blcd motors, with Embassy, or with RTIC 2.x.


## Goals

* Exclusively use fixed-point math for all FOC calculations, using the fixed crate.
* Generic over angle sensors, current sensors, and PWM drivers.
* No heap allocations anywhere.
* Can be used with RTIC 2.0, or Embassy,
* Application layer is fully decoupled from the control layer. In this way the library is easy to use. (No owning issues to solve)
* Provides a basic serial command line terminal for controlling the motors
* Multiple motors can be controlled
* Able to do the startup calibration of the motor
* reuse library foc v0.3.0 as much as possible


The library does provide interfaces for:
* angle sensor
* current sensor
* serial communication
* pwm driver

The user of this library should implement these interfaces. The user will have to solve all the nitty gritty details for controlling the hardware.

In this way the library can stay very generic. 

This library provides:
* run the motor in angle mode. Set the angle in radians
* run the motor in torque mode. Set the torque with a value from 0..1
* run the motor in velocity mode. Set the speed in radians/sec
* Set the acceleration in velodity mode. Set the acceleration in radians/sec2
* A command line application with help function

This library does NOT provide:
* current sensor implementation. Although current sense implementation can be done easy, as the library is already prepared for this. See the update function in FocPwm object.


# Layers

The library does contain the following layers
* The trait definititions to be implemented by the user
* A fully synchronous layer, with does have no dependencies towards rtic or embassy.
* An async wrapping layer for embassy
* An async wrapping layer for rtic 2.0
* Examples for controlling motors, with embassy or rtic, with 2 different platforms (nucleo-f103rb and storm32). Examples are available in the github repository



## Buiding this library

To build this library feature "rtic" or "embassy" must be selected:

* cargo build --release --features="rtic"

or
*  cargo build --release --features="embassy" 
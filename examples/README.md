# foc_simple_test

Tests are done with 2 boards
* storm32 v1.31    gimbal board for 3 small gimbal motors
* nucleo-f103rb + x-nucleo-IHMO7m1 hat

# Storm32 

Connected to this board are 2 2804H small Gimbal motors. For feedback an as5600 magnetic encoder board is mounded on the back of the motor. 

This board can be used as is. No modifications needed.
For the shell tests one needs to solder a hc08 bluetooth module on the back.

The left motor in the test is connected without angle sensor. 
Sensorless mode is tested here.

The right motor in the test does have an AS5600 angle sensor,connected via an I2C.

Note: This setup is not stable for me, possible due to cheap Chinese storm32 board. The i2c bus does get blocked after a while. I am working on a solution with does measure the digital output pulse length from the out pin of the AS5600.


## Nucleo nucleo-f103rb

Connected to this board is a heavy 24 volt BLCD motor with planetary gear box, and digital HALL sensors as feedback.

This board and the Hat need some modification to get it working.

On the nucleo board:
* Remove SB15 zero ohm resistor. This will disconnect  the SWO signal of the CN4 connector to PB3. PB3 is connected to one of the hall sensors

On the x-nucleo-IHMO7m1 Hat:
* Remove R82. This will break the connection between the current feedback 2 and PB13. PB13 can now be freely used
* Remove R49. This will break the connection between the chip enable and PB14. PB14 can be now freely used.
* Connect a jumper wire from PB3 (hall sensor 2) to PB14.

Now all hall sensors can be handled by one interrupt (EXTI15_10). This is important in RTIC, as now the hall sensor can be a local dependency, and no locking is needed to access it.

The push button on the nucleo board can be used now (connected to PC13), if needed. In RTIC it will generate the EXTI15_10 interupt.

## Build and run

To buils an run an example in the src/bin folder one has to:
* connect the board with an stlink probe
* step into one of the example dirs (eg: cd rtic-f103-cb)
* check the runner in ./cargo/config.toml 
* cargo run --release --bin test-shell

## Available tests

| binary | description | comment |
|  --- | --- | ---  |
|test-serial|Basic low level test of serial  port.| Only needed to check serial communication|
|test-pwm|Test the FocPwm object | Only the last step in foc flow is tested: the current loop and the PWM generation from the desired torque and flux  |
|test-velocity|Test FOC in velocity mode with angle sensor feedback| 
test-shell-sec|Test the command line user interface, with no FOC configured, and loaded| Does show that the user interface can even run without any active FOC. User interface is fully decoupled from ccontrol layer
|test-shell|Test the foc library with the command line user interface|Full operation of foc motors via the serial command line interface|
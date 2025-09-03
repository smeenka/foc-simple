# foc_simple_test

Tests are done with 2 boards
* storm32 v1.31    gimbal board for 3 small gimbal motors
* nucleo-f103rb + x-nucleo-IHMO7m1 hat

# Storm32 
This board can be used as is. No modifications needed.
For the shell tests one needs to solder a hc08 bluetooth module on the back.


## Nucleo nucleo-f103rb

This board and the hat need some modification to get it working.

On the nucleo board:
* Remove SB15 zero ohm resistor. This will disconnect  the SWO signal of the CN4 connector to PB3. PB3 is connected to one of the hall sensors

On the x-nucleo-IHMO7m1 hat:
* Remove R82. This will break the connection between the current feedback 2 and PB13. PB13 can now be freely used
* Remove R49. This will break the connection between the chip enable and PB14. PB14 can be now freely used.
* Connect a jumper wire from PB3 (hall sensor 2) to PB14.

Now all hall sensors can be handled by one interrupt (EXTI15_10). This is important in RTIC, as now the hall sensor can be a local dependency, and no locking is needed to access it.

The push button on the nucleo board can be used now (connected to PC13), if needed. In RTIC it will generate the EXTI15_10 interupt.

## Build and run

To buils an run an example in the src/bin folder one has to:
* connect the board with an stlink debugger
* step into one of the example dirs (eg: cd rtic-f103-cb)
* check the runner in ./cargo/config.toml 
* cargo run --release --bin test-shell

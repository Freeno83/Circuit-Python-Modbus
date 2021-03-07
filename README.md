# Circuit-Python-Modbus
Circuit Python compatible version of minimal modbus

Functionality included is for an RTU Client (master) over RS-485 with the following functions:

1) read_bits
2) write_bits
3) read_regsiters
4) write_registers
5) read_float
6) write_float
7) read_long
8) write_long

The ability to read/write strings as well as ASCII protocol has been removed compared to minimal modbus.

the read and write functions for bits/registers an be used for single or multiple bits/registers.

Example use is shown in the attached code file.

These further modifications will be made in the next few weeks:

1) Simplify and reduce where possible the number of value checks
2) Make it so that any function call always returns a value so that the success of the command can be judged.  This would be similar to how the arduino modbus library does i.e. in case of a read return the value on success or -1 in case of failure.  In case of a write, 1 on success, 0 on failure.  

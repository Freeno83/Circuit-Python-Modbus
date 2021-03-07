from CPModbus import Instrument

# https://www.automationdirect.com/adc/overview/catalog/programmable_controllers/click_series_plcs/click_plcs_(stackable_micro_brick)
plc = Instrument(1, baudrate=115200)

plc.write_bits(0x2000, [1]*6) # Set all outputs high
print(plc.read_bits(0x2000, 6)) # read the state of all outputs

plc.write_registers(0x0000, [777, 888]) # write values to 2 registers
print(plc.read_registers(0x0000, 2)) # read values from 2 registers

plc.write_long(0x4000, 555555555) # Write a long value
print(plc.read_long(0x4000)) # read from a double register

plc.write_float(0x7000, 11.22) # write a cloat value
print(plc.read_float(0x7000)) # read a float value

from CPModbus import Instrument

# https://www.automationdirect.com/adc/overview/catalog/programmable_controllers/click_series_plcs/click_plcs_(stackable_micro_brick)
plc = Instrument(1, baudrate=115200)

plc.write_bits(0x2000, [1]*6)
print(plc.read_bits(0x2000, 6))

plc.write_registers(0x0000, [777, 888])
print(plc.read_registers(0x0000, 2))

plc.write_long(0x4000, 555555555)
print(plc.read_long(0x4000))

plc.write_float(0x7000, 11.22)
print(plc.read_float(0x7000))

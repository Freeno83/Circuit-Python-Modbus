# Circuit Python Modbus
# A library with RTU, Client, and RS-485 capability
# Works for numerical values only, no text capability
# Derived from: https://minimalmodbus.readthedocs.io/en/stable/
# Written by Nicholas Robinson

import os
import struct
import sys
import time
import binascii
long = int

import busio
import board
from digitalio import DigitalInOut, Pull, Direction

_NUMBER_OF_BYTES_BEFORE_REGISTERDATA = 1
_NUMBER_OF_BYTES_PER_REGISTER = 2
_MAX_NUMBER_OF_REGISTERS_TO_WRITE = 123
_MAX_NUMBER_OF_REGISTERS_TO_READ = 125
_MAX_NUMBER_OF_BITS_TO_WRITE = 1968
_MAX_NUMBER_OF_BITS_TO_READ = 2000
_MAX_NUMBER_OF_DECIMALS = 10
_MAX_BYTEORDER_VALUE = 3
_SECONDS_TO_MILLISECONDS = 1000
_BITS_PER_BYTE = 8
_BYTEPOSITION_FOR_SLAVEADDRESS = 0
_BYTEPOSITION_FOR_FUNCTIONCODE = 1
_BYTEPOSITION_FOR_SLAVE_ERROR_CODE = 2
_BITNUMBER_FUNCTIONCODE_ERRORINDICATION = 7

MODE_RTU = "rtu"
BYTEORDER_BIG = 0
BYTEORDER_LITTLE = 1
BYTEORDER_BIG_SWAP = 2
BYTEORDER_LITTLE_SWAP = 3

_PAYLOADFORMAT_BIT = "bit"
_PAYLOADFORMAT_BITS = "bits"
_PAYLOADFORMAT_FLOAT = "float"
_PAYLOADFORMAT_LONG = "long"
_PAYLOADFORMAT_REGISTER = "register"
_PAYLOADFORMAT_REGISTERS = "registers"
_ALL_PAYLOADFORMATS = [
    _PAYLOADFORMAT_BIT,
    _PAYLOADFORMAT_BITS,
    _PAYLOADFORMAT_FLOAT,
    _PAYLOADFORMAT_LONG,
    _PAYLOADFORMAT_REGISTER,
    _PAYLOADFORMAT_REGISTERS]


class Instrument:
    """Class for talking to slave devices with RTU and RS-485"""
    def __init__(
        self,
        slaveaddress=1,
        baudrate=9600,
        parity=busio.UART.Parity.ODD,
        stop=1):
        """Initialize instrument and open corresponding serial port"""

        self.address = slaveaddress
        self.mode = MODE_RTU
        self.prev_time = 0.0

        if "IO7" in dir(board):
            self.rs485_ctrl = DigitalInOut(board.IO7) # ESP32s2
        else:
            self.rs485_ctrl = DigitalInOut(board.D2) # Metro Express M0

        self.rs485_ctrl.direction = Direction.OUTPUT
        self.rs485_ctrl.value = False

        self.serial = busio.UART(tx=board.TX,
                                 rx=board.RX,
                                 baudrate=baudrate,
                                 bits=8,
                                 parity=parity,
                                 stop=stop,
                                 timeout=2.0)

    def read_bits(self, registeraddress, number_of_bits=1):
        """Read multiple bits from the slave (instrument)"""

        _check_int(
            number_of_bits,
            minvalue=1,
            maxvalue=_MAX_NUMBER_OF_BITS_TO_READ,
            description="number of bits")

        return self._generic_command(
            functioncode=1,
            registeraddress=registeraddress,
            number_of_bits=number_of_bits,
            payloadformat=_PAYLOADFORMAT_BITS)

    def write_bits(self, registeraddress, values):
        """Write multiple bits to the slave (instrument)"""

        if not isinstance(values, list):
            raise TypeError(
                'The "values parameter" must be a list. Given: {}'.format(values))

        _check_int(
            len(values),
            minvalue=1,
            maxvalue=_MAX_NUMBER_OF_BITS_TO_WRITE,
            description="length of input list")

        self._generic_command(
            functioncode=15,
            registeraddress=registeraddress,
            value=values,
            number_of_bits=len(values),
            payloadformat=_PAYLOADFORMAT_BITS)

    def read_registers(self, registeraddress, number_of_registers=1):
        """Read integers from 16-bit registers in the slave"""

        _check_int(
            number_of_registers,
            minvalue=1,
            maxvalue=_MAX_NUMBER_OF_REGISTERS_TO_READ,
            description="number of registers")

        return self._generic_command(
            functioncode=3,
            registeraddress=registeraddress,
            number_of_registers=number_of_registers,
            payloadformat=_PAYLOADFORMAT_REGISTERS)

    def write_registers(self, registeraddress, values):
        """Write integers to 16-bit registers in the slave"""

        if not isinstance(values, list):
            raise TypeError(
                'The "values parameter" must be a list. Given: {0!r}'.format(values))

        _check_int(
            len(values),
            minvalue=1,
            maxvalue=_MAX_NUMBER_OF_REGISTERS_TO_WRITE,
            description="length of input list")

        self._generic_command(
            functioncode=16,
            registeraddress=registeraddress,
            value=values,
            number_of_registers=len(values),
            payloadformat=_PAYLOADFORMAT_REGISTERS)

    def read_float(
    	self,
    	registeraddress,
    	number_of_registers=2,
        byteorder=BYTEORDER_BIG_SWAP):
        """Read a floating point number from the slave"""

        _check_int(
            number_of_registers,
            minvalue=2,
            maxvalue=4,
            description="number of registers")

        return self._generic_command(
            functioncode=3,
            registeraddress=registeraddress,
            number_of_registers=number_of_registers,
            byteorder=byteorder,
            payloadformat=_PAYLOADFORMAT_FLOAT)

    def write_float(
        self, registeraddress, value, number_of_registers=2, byteorder=BYTEORDER_BIG_SWAP):
        """Write a floating point number to the slave"""

        _check_numerical(value, description="input value")
        _check_int(
            number_of_registers,
            minvalue=2,
            maxvalue=4,
            description="number of registers")

        self._generic_command(
            functioncode=16,
            registeraddress=registeraddress,
            value=value,
            number_of_registers=number_of_registers,
            byteorder=byteorder,
            payloadformat=_PAYLOADFORMAT_FLOAT)

    def read_long(self, registeraddress, signed=False, byteorder=BYTEORDER_BIG_SWAP):
        """Read a long integer (32 bits) from the slave"""

        _check_bool(signed, description="signed")

        return self._generic_command(
            functioncode=3,
            registeraddress=registeraddress,
            number_of_registers=2,
            signed=signed,
            byteorder=byteorder,
            payloadformat=_PAYLOADFORMAT_LONG)

    def write_long(self, registeraddress, value, signed=False, byteorder=BYTEORDER_BIG_SWAP):
        """Write a long integer (32 bits) to the slave"""

        MAX_VALUE_LONG = 4294967295  # Unsigned INT32
        MIN_VALUE_LONG = -2147483648  # INT32

        _check_int(
            value,
            minvalue=MIN_VALUE_LONG,
            maxvalue=MAX_VALUE_LONG,
            description="input value")

        _check_bool(signed, description="signed")

        self._generic_command(
            functioncode=16,
            registeraddress=registeraddress,
            value=value,
            number_of_registers=2,
            signed=signed,
            byteorder=byteorder,
            payloadformat=_PAYLOADFORMAT_LONG)

    def _generic_command(
        self,
        functioncode,
        registeraddress,
        value=None,
        number_of_decimals=0,
        number_of_registers=0,
        number_of_bits=0,
        signed=False,
        byteorder=BYTEORDER_BIG,
        payloadformat=None):
        """Perform generic command for reading and writing registers and bits"""

        _check_registeraddress(registeraddress)
        _check_int(
            number_of_decimals,
            minvalue=0,
            maxvalue=_MAX_NUMBER_OF_DECIMALS,
            description="number of decimals")

        _check_int(
            number_of_registers,
            minvalue=0,
            maxvalue=max(
                _MAX_NUMBER_OF_REGISTERS_TO_READ, _MAX_NUMBER_OF_REGISTERS_TO_WRITE),
            description="number of registers")

        _check_int(
            number_of_bits,
            minvalue=0,
            maxvalue=max(_MAX_NUMBER_OF_BITS_TO_READ, _MAX_NUMBER_OF_BITS_TO_WRITE),
            description="number of bits")

        _check_bool(signed, description="signed")
        _check_int(
            byteorder,
            minvalue=0,
            maxvalue=_MAX_BYTEORDER_VALUE,
            description="byteorder")

        # Create payload
        payload_to_slave = _create_payload(
            functioncode,
            registeraddress,
            value,
            number_of_decimals,
            number_of_registers,
            number_of_bits,
            signed,
            byteorder,
            payloadformat)

     	# Communicate with instrument
        payload_from_slave = self._perform_command(functioncode, payload_to_slave)

        # Parse response payload
        return _parse_payload(
            payload_from_slave,
            functioncode,
            registeraddress,
            value,
            number_of_decimals,
            number_of_registers,
            number_of_bits,
            signed,
            byteorder,
            payloadformat)

    def _perform_command(self, functioncode, payload_to_slave):
        """Perform the command having the *functioncode* """

        _check_functioncode(functioncode, None)

        request = _embed_payload(
            self.address, self.mode, functioncode, payload_to_slave)

        number_of_bytes_to_read = _predict_response_size(
                self.mode, functioncode, payload_to_slave)

        response = self._communicate(request, number_of_bytes_to_read)

        payload_from_slave = _extract_payload(
            response, self.address, self.mode, functioncode)

        return payload_from_slave

    def _communicate(self, request, number_of_bytes_to_read):
        """Talk to the slave via a serial port"""

        _check_int(number_of_bytes_to_read)

        minimum_silent_period = _calculate_minimum_silent_period(self.serial.baudrate)
        time_since_read = time.monotonic() - self.prev_time

        if time_since_read < minimum_silent_period:
            sleep_time = minimum_silent_period - time_since_read
            time.sleep(sleep_time)

        self.prev_time = time.monotonic()
        #print("Request: {}".format(_hexlify(request)))

        self.rs485_ctrl.value = True
        self.serial.reset_input_buffer()
        self.serial.write(request)
        self.rs485_ctrl.value = False
        self.prev_time = time.monotonic()

        answer = self.serial.read(number_of_bytes_to_read)
        #print("Answer: {}".format(_hexlify(answer)))

        if not answer:
            raise ValueError("No communication with the instrument (no answer)")

        return answer

def _create_payload(
    functioncode,
    registeraddress,
    value,
    number_of_decimals,
    number_of_registers,
    number_of_bits,
    signed,
    byteorder,
    payloadformat):
    """Create the payload"""

    if functioncode in [1, 2]:
        return _num_to_twobyte_string(registeraddress) + _num_to_twobyte_string(
            number_of_bits)

    if functioncode in [3, 4]:
        return _num_to_twobyte_string(registeraddress) + _num_to_twobyte_string(
            number_of_registers)

    if functioncode == 15:
        if payloadformat == _PAYLOADFORMAT_BIT:
            bitlist = [value]
        else:
            bitlist = value
        return (
            _num_to_twobyte_string(registeraddress)
            + _num_to_twobyte_string(number_of_bits)
            + _num_to_onebyte_string(
                _calculate_number_of_bytes_for_bits(number_of_bits))
            + _bits_to_bytestring(bitlist))

    if functioncode == 16:
        if payloadformat == _PAYLOADFORMAT_REGISTER:
            registerdata = _num_to_twobyte_string(
                value, number_of_decimals, signed=signed)

        elif payloadformat == _PAYLOADFORMAT_LONG:
            registerdata = _long_to_bytestring(
                value, signed, number_of_registers, byteorder)

        elif payloadformat == _PAYLOADFORMAT_FLOAT:
            registerdata = _float_to_bytestring(value, number_of_registers, byteorder)
        elif payloadformat == _PAYLOADFORMAT_REGISTERS:
            registerdata = _valuelist_to_bytestring(value, number_of_registers)

        return (
            _num_to_twobyte_string(registeraddress)
            + _num_to_twobyte_string(number_of_registers)
            + _num_to_onebyte_string(len(registerdata))
            + registerdata)

    raise ValueError("Wrong function code: " + str(functioncode))

def _parse_payload(
    payload,
    functioncode,
    registeraddress,
    value,
    number_of_decimals,
    number_of_registers,
    number_of_bits,
    signed,
    byteorder,
    payloadformat):

    _check_response_payload(
        payload,
        functioncode,
        registeraddress,
        value,
        number_of_decimals,
        number_of_registers,
        number_of_bits,
        signed,
        byteorder,
        payloadformat)

    if functioncode in [1, 2]:
        registerdata = payload[_NUMBER_OF_BYTES_BEFORE_REGISTERDATA:]
        if payloadformat == _PAYLOADFORMAT_BIT:
            return _bytestring_to_bits(registerdata, number_of_bits)[0]
        elif payloadformat == _PAYLOADFORMAT_BITS:
            return _bytestring_to_bits(registerdata, number_of_bits)

    if functioncode in [3, 4]:
        registerdata = payload[_NUMBER_OF_BYTES_BEFORE_REGISTERDATA:]

        if payloadformat == _PAYLOADFORMAT_LONG:
            return _bytestring_to_long(
                registerdata, signed, number_of_registers, byteorder)

        elif payloadformat == _PAYLOADFORMAT_FLOAT:
            return _bytestring_to_float(registerdata, number_of_registers, byteorder)

        elif payloadformat == _PAYLOADFORMAT_REGISTERS:
            return _bytestring_to_valuelist(registerdata, number_of_registers)

        elif payloadformat == _PAYLOADFORMAT_REGISTER:
            return _twobyte_string_to_num(
                registerdata, number_of_decimals, signed=signed)

def _embed_payload(slaveaddress, mode, functioncode, payloaddata):
    """Build a request from the slaveaddress, the function code and the payload data"""

    _check_slaveaddress(slaveaddress)
    _check_mode(mode)

    first_part = b""
    first_part += _num_to_onebyte_string(slaveaddress)
    first_part += _num_to_onebyte_string(functioncode)
    first_part += payloaddata

    return first_part + _calculate_crc_string(first_part)

def _extract_payload(response, slaveaddress, mode, functioncode):
    """Extract the payload data part from the slave's response"""

    NUMBER_OF_RESPONSE_STARTBYTES = 2
    NUMBER_OF_CRC_BYTES = 2
    MINIMAL_RESPONSE_LENGTH_RTU = NUMBER_OF_RESPONSE_STARTBYTES + NUMBER_OF_CRC_BYTES

    _check_slaveaddress(slaveaddress)
    _check_mode(mode)
    _check_functioncode(functioncode, None)

    plainresponse = response

    if len(response) < MINIMAL_RESPONSE_LENGTH_RTU:
        raise ValueError(
            "Too short Modbus RTU response (minimum length {} bytes). Response: {!r}".format(
                MINIMAL_RESPONSE_LENGTH_RTU, response))

    calculate_checksum = _calculate_crc_string
    number_of_checksum_bytes = NUMBER_OF_CRC_BYTES

    received_checksum = response[-number_of_checksum_bytes:]
    response_without_checksum = response[0 : (len(response) - number_of_checksum_bytes)]
    calculated_checksum = calculate_checksum(response_without_checksum)

    if received_checksum != calculated_checksum:
        template = (
            "Checksum error in {} mode: {!r} instead of {!r} . The response "
            + "is: {!r} (plain response: {!r})")

        text = template.format(
            mode, received_checksum, calculated_checksum, response, plainresponse)

        raise ValueError(text)

    responseaddress = response[_BYTEPOSITION_FOR_SLAVEADDRESS]

    if responseaddress != slaveaddress:
        raise ValueError(
            "Wrong return slave address: {} instead of {}. The response is: {!r}".format(
                responseaddress, slaveaddress, response))

    _check_response_slaveerrorcode(response)

    received_functioncode = response[_BYTEPOSITION_FOR_FUNCTIONCODE]
    if received_functioncode != functioncode:
        raise ValueError(
            "Wrong functioncode: {} instead of {}. The response is: {!r}".format(
                received_functioncode, functioncode, response))

    first_databyte_number = NUMBER_OF_RESPONSE_STARTBYTES
    last_databyte_number = len(response) - NUMBER_OF_CRC_BYTES

    return response[first_databyte_number:last_databyte_number]

def _predict_response_size(mode, functioncode, payload_to_slave):
    """Calculate the number of bytes that should be received from the slave"""

    MIN_PAYLOAD_LENGTH = 4
    BYTERANGE_FOR_GIVEN_SIZE = slice(2, 4)

    NUMBER_OF_PAYLOAD_BYTES_IN_WRITE_CONFIRMATION = 4
    NUMBER_OF_PAYLOAD_BYTES_FOR_BYTECOUNTFIELD = 1

    NUMBER_OF_RTU_RESPONSE_STARTBYTES = 2
    NUMBER_OF_RTU_RESPONSE_ENDBYTES = 2


    _check_mode(mode)
    _check_functioncode(functioncode, None)

    if functioncode in [5, 6, 15, 16]:
        response_payload_size = NUMBER_OF_PAYLOAD_BYTES_IN_WRITE_CONFIRMATION

    elif functioncode in [1, 2, 3, 4]:
        given_size = _twobyte_string_to_num(payload_to_slave[BYTERANGE_FOR_GIVEN_SIZE])
        if functioncode in [1, 2]:
            number_of_inputs = given_size
            response_payload_size = (
                NUMBER_OF_PAYLOAD_BYTES_FOR_BYTECOUNTFIELD
                + number_of_inputs // 8
                + (1 if number_of_inputs % 8 else 0))

        elif functioncode in [3, 4]:
            number_of_registers = given_size
            response_payload_size = (
                NUMBER_OF_PAYLOAD_BYTES_FOR_BYTECOUNTFIELD
                + number_of_registers * _NUMBER_OF_BYTES_PER_REGISTER)
    else:
        raise ValueError(
            "Wrong functioncode: {}. The payload is: {!r}".format(
                functioncode, payload_to_slave))

    return (
    	NUMBER_OF_RTU_RESPONSE_STARTBYTES
    	+ response_payload_size
    	+ NUMBER_OF_RTU_RESPONSE_ENDBYTES)

def _calculate_minimum_silent_period(baudrate):
    """Calculate the silent period length between messages"""

    _check_numerical(baudrate, minvalue=1, description="baudrate")

    BITTIMES_PER_CHARACTERTIME = 11
    MINIMUM_SILENT_CHARACTERTIMES = 3.5
    MINIMUM_SILENT_TIME_SECONDS = 0.00175

    bittime = 1 / float(baudrate)
    return max(
        bittime * BITTIMES_PER_CHARACTERTIME * MINIMUM_SILENT_CHARACTERTIMES,
        MINIMUM_SILENT_TIME_SECONDS)

def _num_to_onebyte_string(inputvalue):
    """Convert a numerical value to a one-byte string"""

    _check_int(inputvalue, minvalue=0, maxvalue=0xFF)
    return chr(inputvalue)

def _num_to_twobyte_string(value, number_of_decimals=0, lsb_first=False, signed=False):
    """Convert a numerical value to a two-byte string, possibly scaling it"""

    _check_numerical(value, description="inputvalue")

    if number_of_decimals is not 0:
	    _check_int(
	        number_of_decimals,
	        minvalue=0,
	        maxvalue=_MAX_NUMBER_OF_DECIMALS,
	        description="number of decimals")

    _check_bool(lsb_first, description="lsb_first")
    _check_bool(signed, description="signed parameter")

    multiplier = 10 ** number_of_decimals
    integer = int(float(value) * multiplier)

    if lsb_first:
        formatcode = "<"
    else:
        formatcode = ">"
    if signed:
        formatcode += "h"
    else:
        formatcode += "H"

    return _pack(formatcode, integer)

def _twobyte_string_to_num(bytestring, number_of_decimals=0, signed=False):
    """Convert a two-byte string to a numerical value, possibly scaling it"""

    if number_of_decimals is not 0:
	    _check_int(
	        number_of_decimals,
	        minvalue=0,
	        maxvalue=_MAX_NUMBER_OF_DECIMALS,
	        description="number of decimals")

    _check_bool(signed, description="signed parameter")

    formatcode = ">"
    if signed:
        formatcode += "h"
    else:
        formatcode += "H"

    fullregister = _unpack(formatcode, bytestring)

    if number_of_decimals == 0:
        return fullregister
    divisor = 10 ** number_of_decimals
    return fullregister / float(divisor)

def _long_to_bytestring(
    value, signed=False, number_of_registers=2, byteorder=BYTEORDER_BIG):
    """Convert a long integer to a bytestring"""

    _check_int(value, description="inputvalue")
    _check_bool(signed, description="signed parameter")
    _check_int(
        number_of_registers, minvalue=2, maxvalue=2, description="number of registers")
    _check_int(
        byteorder, minvalue=0, maxvalue=_MAX_BYTEORDER_VALUE, description="byteorder")

    if byteorder in [BYTEORDER_BIG, BYTEORDER_BIG_SWAP]:
        formatcode = ">"
    else:
        formatcode = "<"
    if signed:
        formatcode += "l"
    else:
        formatcode += "L"

    outstring = _pack(formatcode, value)
    if byteorder in [BYTEORDER_BIG_SWAP, BYTEORDER_LITTLE_SWAP]:
        outstring = _swap(outstring)

    assert len(outstring) == 4
    return outstring

def _bytestring_to_long(
    bytestring, signed=False, number_of_registers=2, byteorder=BYTEORDER_BIG):
    """Convert a bytestring to a long integer"""

    _check_bool(signed, description="signed parameter")
    _check_int(
        number_of_registers, minvalue=2, maxvalue=2, description="number of registers")
    _check_int(
        byteorder, minvalue=0, maxvalue=_MAX_BYTEORDER_VALUE, description="byteorder")

    if byteorder in [BYTEORDER_BIG, BYTEORDER_BIG_SWAP]:
        formatcode = ">"
    else:
        formatcode = "<"
    if signed:
        formatcode += "l"
    else:
        formatcode += "L"

    if byteorder in [BYTEORDER_BIG_SWAP, BYTEORDER_LITTLE_SWAP]:
        bytestring = _swap(bytestring)

    return _unpack(formatcode, bytestring)

def _float_to_bytestring(value, number_of_registers=2, byteorder=BYTEORDER_BIG):
    """Convert a numerical value to a bytestring"""

    _check_numerical(value, description="inputvalue")
    _check_int(
        number_of_registers, minvalue=2, maxvalue=4, description="number of registers")
    _check_int(
        byteorder, minvalue=0, maxvalue=_MAX_BYTEORDER_VALUE, description="byteorder")

    if byteorder in [BYTEORDER_BIG, BYTEORDER_BIG_SWAP]:
        formatcode = ">"
    else:
        formatcode = "<"
    if number_of_registers == 2:
        formatcode += "f"
        lengthtarget = 4
    elif number_of_registers == 4:
        formatcode += "d"
        lengthtarget = 8
    else:
        raise ValueError(
            "Wrong number of registers! Given value is {0!r}".format(
                number_of_registers))

    outstring = _pack(formatcode, value)
    if byteorder in [BYTEORDER_BIG_SWAP, BYTEORDER_LITTLE_SWAP]:
        outstring = _swap(outstring)
    assert len(outstring) == lengthtarget
    return outstring

def _bytestring_to_float(bytestring, number_of_registers=2, byteorder=BYTEORDER_BIG):
    """Convert a four-byte string to a float"""

    _check_int(
        number_of_registers, minvalue=2, maxvalue=4, description="number of registers")
    _check_int(
        byteorder, minvalue=0, maxvalue=_MAX_BYTEORDER_VALUE, description="byteorder")

    number_of_bytes = _NUMBER_OF_BYTES_PER_REGISTER * number_of_registers

    if byteorder in [BYTEORDER_BIG, BYTEORDER_BIG_SWAP]:
        formatcode = ">"
    else:
        formatcode = "<"
    if number_of_registers == 2:
        formatcode += "f"
    elif number_of_registers == 4:
        formatcode += "d"
    else:
        raise ValueError(
            "Wrong number of registers! Given value is {0!r}".format(
                number_of_registers))

    if len(bytestring) != number_of_bytes:
        raise ValueError(
            "Wrong length of the byte string! Given value is "
            + "{0!r}, and number_of_registers is {1!r}.".format(
                bytestring, number_of_registers))

    if byteorder in [BYTEORDER_BIG_SWAP, BYTEORDER_LITTLE_SWAP]:
        bytestring = _swap(bytestring)
    return _unpack(formatcode, bytestring)

def _valuelist_to_bytestring(valuelist, number_of_registers):
    """Convert a list of numerical values to a bytestring"""

    MINVALUE = 0
    MAXVALUE = 0xFFFF

    _check_int(number_of_registers, minvalue=1, description="number of registers")

    if not isinstance(valuelist, list):
        raise TypeError(
            "The valuelist parameter must be a list. Given {0!r}.".format(valuelist))

    for value in valuelist:
        _check_int(
            value,
            minvalue=MINVALUE,
            maxvalue=MAXVALUE,
            description="elements in the input value list")

    _check_int(
        len(valuelist),
        minvalue=number_of_registers,
        maxvalue=number_of_registers,
        description="length of the list")

    number_of_bytes = _NUMBER_OF_BYTES_PER_REGISTER * number_of_registers

    bytestring = b""
    for value in valuelist:
        bytestring += _num_to_twobyte_string(value, signed=False)

    assert len(bytestring) == number_of_bytes
    return bytestring

def _bytestring_to_valuelist(bytestring, number_of_registers):
    """Convert a bytestring to a list of numerical values"""

    _check_int(number_of_registers, minvalue=1, description="number of registers")

    number_of_bytes = _NUMBER_OF_BYTES_PER_REGISTER * number_of_registers

    values = []
    for i in range(number_of_registers):
        offset = _NUMBER_OF_BYTES_PER_REGISTER * i
        substring = bytestring[offset : (offset + _NUMBER_OF_BYTES_PER_REGISTER)]
        values.append(_twobyte_string_to_num(substring))

    return values

def _pack(formatstring, value):
    """Pack a value into a bytestring"""

    _check_string(formatstring, description="formatstring", minlength=1)

    try:
        result = struct.pack(formatstring, value)
    except Exception:
        errortext = (
            "The value to send is probably out of range, as the num-to-bytestring ")

        errortext += "conversion failed. Value: {0!r} Struct format code is: {1}"
        raise ValueError(errortext.format(value, formatstring))

    return result

def _unpack(formatstring, packed):
    """Unpack a bytestring into a value"""

    try:
        value = struct.unpack(formatstring, packed)[0]
    except Exception:
        errortext = (
            "The received bytestring is probably wrong, as the bytestring-to-num ")

        errortext += "conversion failed. Bytestring: {0!r} Struct format code is: {1}"
        raise ValueError(errortext.format(packed, formatstring))

    return value

def _swap(bytestring):
    """Swap 2xMSB bytes with 2xLSB"""

    length = len(bytestring)
    if length % 2:
        raise ValueError(
            "The length of the bytestring should be even. Given {!r}.".format(
                bytestring))

    return bytestring[2:] + bytestring[:2]

def _hexencode(bytestring, insert_spaces=False):
    """Convert a byte string to a hex encoded string"""

    separator = "" if not insert_spaces else " "

    byte_representions = []
    for char in bytestring:
        byte_representions.append(hex(char))

    return separator.join(byte_representions).strip()

def _hexdecode(hexstring):
    """Convert a hex encoded string to a byte string"""

    _check_string(hexstring, description="hexstring")

    if len(hexstring) % 2 != 0:
        raise ValueError(
            "The input hexstring must be of even length. Given: {!r}".format(hexstring))

    converted_bytes = bytes(hexstring, "latin1")

    try:
        return str(binascii.unhexlify(converted_bytes), encoding="latin1")
    except binascii.Error as err:
        new_error_message = "Hexdecode reported an error: {!s}. Input hexstring: {}".format(
            err.args[0], hexstring)
        raise TypeError(new_error_message)

def _hexlify(bytestring):
    """Convert a byte string to a hex encoded string, with spaces for easier reading"""

    return _hexencode(bytestring, insert_spaces=True)

def _calculate_number_of_bytes_for_bits(number_of_bits):
    """Calculate number of full bytes required to house a number of bits"""

    result = number_of_bits // _BITS_PER_BYTE
    if number_of_bits % _BITS_PER_BYTE:
        result += 1
    return result

def _bit_to_bytestring(value):
    """Create the bit pattern that is used for writing single bits"""

    _check_int(value, minvalue=0, maxvalue=1, description="inputvalue")

    if value == 0:
        return "\x00\x00"
    else:
        return "\xff\x00"

def _bits_to_bytestring(valuelist):
    """Build a bytestring from a list of bits"""

    if not isinstance(valuelist, list):
        raise TypeError(
            "The input should be a list. " + "Given: {!r}".format(valuelist))

    for value in valuelist:
        if value not in [0, 1, False, True]:
            raise ValueError(
                "Wrong value in list of bits. " + "Given: {!r}".format(value))

    list_position = 0
    outputstring = ""
    while list_position < len(valuelist):
        sublist = valuelist[list_position : (list_position + _BITS_PER_BYTE)]

        bytevalue = 0
        for bitposition, value in enumerate(sublist):
            bytevalue |= value << bitposition
        outputstring += chr(bytevalue)

        list_position += _BITS_PER_BYTE
    return outputstring

def _bytestring_to_bits(bytestring, number_of_bits):
    """Parse bits from a bytestring"""

    expected_length = _calculate_number_of_bytes_for_bits(number_of_bits)
    if len(bytestring) != expected_length:
        raise ValueError(
            "Wrong length of bytestring. Expected is "
            + "{} bytes (for {} bits), actual is {} bytes.".format(
                expected_length, number_of_bits, len(bytestring)))

    total_list = []
    for byte in bytestring:
        for bitposition in range(_BITS_PER_BYTE):
            bitvalue = (byte & (1 << bitposition)) > 0
            total_list.append(int(bitvalue))

    return total_list[:number_of_bits]

def _twos_complement(x, bits=16):
    """Calculate the two's complement of an integer"""

    _check_int(bits, minvalue=0, description="number of bits")
    _check_int(x, description="input")
    upperlimit = 2 ** (bits - 1) - 1
    lowerlimit = -2 ** (bits - 1)
    if x > upperlimit or x < lowerlimit:
        raise ValueError(
            "The input value is out of range. Given value is "
            + "{0}, but allowed range is {1} to {2} when using {3} bits.".format(
                x, lowerlimit, upperlimit, bits))

    if x >= 0:
        return x
    return x + 2 ** bits

def _from_twos_complement(x, bits=16):
    """Calculate the inverse(?) of a two's complement of an integer"""

    _check_int(bits, minvalue=0, description="number of bits")
    _check_int(x, description="input")

    upperlimit = 2 ** (bits) - 1
    lowerlimit = 0

    if x > upperlimit or x < lowerlimit:
        raise ValueError(
            "The input value is out of range. Given value is "
            + "{0}, but allowed range is {1} to {2} when using {3} bits.".format(
                x, lowerlimit, upperlimit, bits))

    limit = 2 ** (bits - 1) - 1
    if x <= limit:
        return x
    return x - 2 ** bits

def _set_bit_on(x, bit_num):
    """Set bit 'bit_num' to True"""

    _check_int(x, minvalue=0, description="input value")
    _check_int(bit_num, minvalue=0, description="bitnumber")

    return x | (1 << bit_num)

def _check_bit(x, bit_num):
    """Check if bit 'bit_num' is set the input integer"""

    _check_int(x, minvalue=0, description="input value")
    _check_int(bit_num, minvalue=0, description="bitnumber")

    return (x & (1 << bit_num)) > 0

_CRC16TABLE = (
	0, 49345, 49537, 320, 49921, 960, 640, 49729, 50689, 1728, 1920, 51009,
	1280, 50625, 50305, 1088, 52225, 3264, 3456, 52545, 3840, 53185, 52865,
	3648, 2560, 51905, 52097, 2880, 51457, 2496, 2176, 51265, 55297, 6336,
	6528, 55617, 6912, 56257, 55937, 6720, 7680, 57025, 57217, 8000, 56577,
	7616, 7296, 56385, 5120, 54465, 54657, 5440, 55041, 6080, 5760, 54849,
	53761, 4800, 4992, 54081, 4352, 53697, 53377, 4160, 61441, 12480, 12672,
	61761, 13056, 62401, 62081, 12864, 13824, 63169, 63361, 14144, 62721,
	13760, 13440, 62529, 15360, 64705, 64897, 15680, 65281, 16320, 16000,
	65089, 64001, 15040, 15232, 64321, 14592, 63937, 63617, 14400, 10240,
	59585, 59777, 10560, 60161, 11200, 10880, 59969, 60929, 11968, 12160,
	61249, 11520, 60865, 60545, 11328, 58369, 9408, 9600, 58689, 9984, 59329,
	59009, 9792, 8704, 58049, 58241, 9024, 57601, 8640, 8320, 57409, 40961,
	24768, 24960, 41281, 25344, 41921, 41601, 25152, 26112, 42689, 42881,
	26432, 42241, 26048, 25728, 42049, 27648, 44225, 44417, 27968, 44801,
	28608, 28288, 44609, 43521, 27328, 27520, 43841, 26880, 43457, 43137,
	26688, 30720, 47297, 47489, 31040, 47873, 31680, 31360, 47681, 48641,
	32448, 32640, 48961, 32000, 48577, 48257, 31808, 46081, 29888, 30080,
	46401, 30464, 47041, 46721, 30272, 29184, 45761, 45953, 29504, 45313,
	29120, 28800, 45121, 20480, 37057, 37249, 20800, 37633, 21440, 21120,
	37441, 38401, 22208, 22400, 38721, 21760, 38337, 38017, 21568, 39937,
	23744, 23936, 40257, 24320, 40897, 40577, 24128, 23040, 39617, 39809,
	23360, 39169, 22976, 22656, 38977, 34817, 18624, 18816, 35137, 19200,
	35777, 35457, 19008, 19968, 36545, 36737, 20288, 36097, 19904, 19584,
	35905, 17408, 33985, 34177, 17728, 34561, 18368, 18048, 34369, 33281,
	17088, 17280, 33601, 16640, 33217, 32897, 16448)

def _calculate_crc_string(inputstring):
    """Calculate CRC-16 for Modbus"""

    register = 0xFFFF
    for char in inputstring:
        register = (register >> 8) ^ _CRC16TABLE[(register ^ char) & 0xFF]

    return _num_to_twobyte_string(register, lsb_first=True)

def _check_mode(mode):
    """Check that the Modbus mode is RTU"""

    if not isinstance(mode, str):
        raise TypeError("The {0} should be a string. Given: {1!r}".format("mode", mode))

    if mode is not MODE_RTU:
        raise ValueError(
            "Unreconized Modbus mode given. Must be 'rtu' but {0!r} was given.".format(
                mode))

def _check_functioncode(functioncode, list_of_allowed_values=None):
    """Check that the given functioncode is in the list_of_allowed_values"""

    FUNCTIONCODE_MIN = 1
    FUNCTIONCODE_MAX = 127

    _check_int(
        functioncode, FUNCTIONCODE_MIN, FUNCTIONCODE_MAX, description="functioncode")

    if list_of_allowed_values is None:
        return

    if not isinstance(list_of_allowed_values, list):
        raise TypeError(
            "The list_of_allowed_values should be a list. Given: {0!r}".format(
                list_of_allowed_values))

    for value in list_of_allowed_values:
        _check_int(
            value,
            FUNCTIONCODE_MIN,
            FUNCTIONCODE_MAX,
            description="functioncode inside list_of_allowed_values")

    if functioncode not in list_of_allowed_values:
        raise ValueError(
            "Wrong function code: {0}, allowed values are {1!r}".format(
                functioncode, list_of_allowed_values))

def _check_slaveaddress(slaveaddress):
    """Check that the given slaveaddress is valid"""

    SLAVEADDRESS_MAX = 255
    SLAVEADDRESS_MIN = 0

    _check_int(
        slaveaddress, SLAVEADDRESS_MIN, SLAVEADDRESS_MAX, description="slaveaddress")

def _check_registeraddress(registeraddress):
    """Check that the given registeraddress is valid"""

    REGISTERADDRESS_MAX = 0xFFFF
    REGISTERADDRESS_MIN = 0

    _check_int(
        registeraddress,
        REGISTERADDRESS_MIN,
        REGISTERADDRESS_MAX,
        description="registeraddress")

def _check_response_payload(
    payload,
    functioncode,
    registeraddress,
    value,
    number_of_decimals,
    number_of_registers,
    number_of_bits,
    signed,
    byteorder,
    payloadformat):

    if functioncode in [1, 2, 3, 4]:
        _check_response_bytecount(payload)

    if functioncode in [5, 6, 15, 16]:
        _check_response_registeraddress(payload, registeraddress)

    if functioncode == 5:
        _check_response_writedata(payload, _bit_to_bytestring(value))
    elif functioncode == 6:
        _check_response_writedata(
            payload, _num_to_twobyte_string(value, number_of_decimals, signed=signed))

    elif functioncode == 15:
        _check_response_number_of_registers(payload, number_of_bits)

    elif functioncode == 16:
        _check_response_number_of_registers(payload, number_of_registers)

    if functioncode in [1, 2]:
        registerdata = payload[_NUMBER_OF_BYTES_BEFORE_REGISTERDATA:]
        expected_number_of_bytes = _calculate_number_of_bytes_for_bits(number_of_bits)
        if len(registerdata) != expected_number_of_bytes:
            raise ValueError(
                "The data length is wrong for payloadformat BIT/BITS."
                + " Expected: {} Actual: {}.".format(
                    expected_number_of_bytes, len(registerdata)))

    # Response for read registers
    if functioncode in [3, 4]:
        registerdata = payload[_NUMBER_OF_BYTES_BEFORE_REGISTERDATA:]
        number_of_register_bytes = number_of_registers * _NUMBER_OF_BYTES_PER_REGISTER
        if len(registerdata) != number_of_register_bytes:
            raise ValueError(
                "The register data length is wrong. "
                + "Registerdata: {!r} bytes. Expected: {!r}.".format(
                    len(registerdata), number_of_register_bytes))

def _check_response_slaveerrorcode(response):
    """Check if the slave indicates an error"""

    NON_ERRORS = [5]
    SLAVE_ERRORS = {
        1: "Slave reported illegal function",
        2: "Slave reported illegal data address",
        3: "Slave reported illegal data value",
        4: "Slave reported device failure",
        6: "Slave reported device busy",
        7: "Slave reported negative acknowledge",
        8: "Slave reported memory parity error",
        10: "Slave reported gateway path unavailable",
        11: "Slave reported gateway target device failed to respond"}

    if len(response) < _BYTEPOSITION_FOR_SLAVE_ERROR_CODE + 1:
        return

    received_functioncode = response[_BYTEPOSITION_FOR_FUNCTIONCODE]

    if _check_bit(received_functioncode, _BITNUMBER_FUNCTIONCODE_ERRORINDICATION):
        slave_error_code = response[_BYTEPOSITION_FOR_SLAVE_ERROR_CODE]

        if slave_error_code in NON_ERRORS:
            return

        error = SLAVE_ERRORS.get(
            slave_error_code,
            ValueError(
                "Slave reported error code " + str(slave_error_code)))

        raise error

def _check_response_bytecount(payload):
    """Check that the number of bytes as given in the response is correct"""

    POSITION_FOR_GIVEN_NUMBER = 0
    NUMBER_OF_BYTES_TO_SKIP = 1

    given_number_of_databytes = payload[POSITION_FOR_GIVEN_NUMBER]
    counted_number_of_databytes = len(payload) - NUMBER_OF_BYTES_TO_SKIP

    if given_number_of_databytes != counted_number_of_databytes:
        errortemplate = (
            "Wrong given number of bytes in the response: "
            + "{0}, but counted is {1} as data payload length is {2}."
            + " The data payload is: {3!r}")

        errortext = errortemplate.format(
            given_number_of_databytes,
            counted_number_of_databytes,
            len(payload),
            payload)

        raise ValueError(errortext)

def _check_response_registeraddress(payload, registeraddress):
    """Check that the start adress as given in the response is correct"""

    _check_registeraddress(registeraddress)

    BYTERANGE_FOR_STARTADDRESS = slice(0, 2)

    bytes_for_startaddress = payload[BYTERANGE_FOR_STARTADDRESS]
    received_startaddress = _twobyte_string_to_num(bytes_for_startaddress)

    if received_startaddress != registeraddress:
        raise ValueError(
            "Wrong given write start adress: "
            + "{0}, but commanded is {1}. The data payload is: {2!r}".format(
                received_startaddress, registeraddress, payload))

def _check_response_number_of_registers(payload, number_of_registers):
    """Check that the number of written registers as given in the response is correct"""

    _check_int(
        number_of_registers,
        minvalue=1,
        maxvalue=max(
            _MAX_NUMBER_OF_REGISTERS_TO_READ, _MAX_NUMBER_OF_REGISTERS_TO_WRITE),
        description="number of registers")

    BYTERANGE_FOR_NUMBER_OF_REGISTERS = slice(2, 4)

    bytes_for_mumber_of_registers = payload[BYTERANGE_FOR_NUMBER_OF_REGISTERS]
    received_number_of_written_registers = _twobyte_string_to_num(
        bytes_for_mumber_of_registers)

    if received_number_of_written_registers != number_of_registers:
        raise ValueError(
            "Wrong number of registers to write in the response: "
            + "{0}, but commanded is {1}. The data payload is: {2!r}".format(
                received_number_of_written_registers, number_of_registers, payload))

def _check_response_writedata(payload, writedata):
    """Check that the write data as given in the response is correct"""

    BYTERANGE_FOR_WRITEDATA = slice(2, 4)
    received_writedata = payload[BYTERANGE_FOR_WRITEDATA]

    if received_writedata != writedata:
        raise ValueError(
            "Wrong write data in the response: "
            + "{0!r}, but commanded is {1!r}. The data payload is: {2!r}".format(
                received_writedata, writedata, payload))

def _check_string(
    inputstring,
    description,
    minlength=0,
    maxlength=None,
    force_ascii=False,
    exception_type=ValueError):
    """Check that the given string is valid"""

    if not isinstance(description, str):
        raise TypeError(
            "The description should be a string. Given: {0!r}".format(description))

    if not isinstance(inputstring, str):
        raise TypeError(
            "The {0} should be a string. Given: {1!r}".format(description, inputstring))

    if not isinstance(maxlength, (int, type(None))):
        raise TypeError(
            "The maxlength must be an integer or None. Given: {0!r}".format(maxlength))
    try:
        issubclass(exception_type, Exception)
    except TypeError:
        raise TypeError(
            "The exception_type must be an exception class. "
            + "It not even a class. Given: {0!r}".format(type(exception_type)))
    if not issubclass(exception_type, Exception):
        raise TypeError(
            "The exception_type must be an exception class. Given: {0!r}".format(
                type(exception_type)))

    _check_int(minlength, minvalue=0, maxvalue=None, description="minlength")

    if len(inputstring) < minlength:
        raise exception_type(
            "The {0} is too short: {1}, but minimum value is {2}. Given: {3!r}".format(
                description, len(inputstring), minlength, inputstring))

    if maxlength is not None:
        if maxlength < 0:
            raise ValueError(
                "The maxlength must be positive. Given: {0}".format(maxlength))

        if maxlength < minlength:
            raise ValueError(
                "The maxlength must not be smaller than minlength. Given: {0} and {1}".format(
                    maxlength, minlength))

        if len(inputstring) > maxlength:
            raise exception_type(
                "The {0} is too long: {1}, but maximum value is {2}. Given: {3!r}".format(
                    description, len(inputstring), maxlength, inputstring))

    if force_ascii and sys.version > "3":
        try:
            inputstring.encode("ascii")
        except UnicodeEncodeError:
            raise ValueError(
                "The {0} must be ASCII. Given: {1!r}".format(description, inputstring))

def _check_int(inputvalue, minvalue=None, maxvalue=None, description="inputvalue"):
    """Check that the given integer is valid"""

    if not isinstance(description, str):
        raise TypeError(
            "The description should be a string. Given: {0!r}".format(description))

    if not isinstance(inputvalue, (int, long)):
        raise TypeError(
            "The {0} must be an integer. Given: {1!r}".format(description, inputvalue))

    if not isinstance(minvalue, (int, long, type(None))):
        raise TypeError(
            "The minvalue must be an integer or None. Given: {0!r}".format(minvalue))

    if not isinstance(maxvalue, (int, long, type(None))):
        raise TypeError(
            "The maxvalue must be an integer or None. Given: {0!r}".format(maxvalue))

    _check_numerical(inputvalue, minvalue, maxvalue, description)

def _check_numerical(
    inputvalue, minvalue=None, maxvalue=None, description="inputvalue"):
    """Check that the given numerical value is valid"""

    if not isinstance(description, str):
        raise TypeError(
            "The description should be a string. Given: {0!r}".format(description))

    if not isinstance(inputvalue, (int, long, float)):
        raise TypeError(
            "The {0} must be numerical. Given: {1!r}".format(description, inputvalue))

    if not isinstance(minvalue, (int, float, long, type(None))):
        raise TypeError(
            "The minvalue must be numeric or None. Given: {0!r}".format(minvalue))

    if not isinstance(maxvalue, (int, float, long, type(None))):
        raise TypeError(
            "The maxvalue must be numeric or None. Given: {0!r}".format(maxvalue))

    if (minvalue is not None) and (maxvalue is not None):
        if maxvalue < minvalue:
            raise ValueError(
                "The maxvalue must not be smaller than minvalue. "
                + "Given: {0} and {1}, respectively.".format(maxvalue, minvalue))

    if minvalue is not None:
        if inputvalue < minvalue:
            raise ValueError(
                "The {0} is too small: {1}, but minimum value is {2}.".format(
                    description, inputvalue, minvalue))

    if maxvalue is not None:
        if inputvalue > maxvalue:
            raise ValueError(
                "The {0} is too large: {1}, but maximum value is {2}.".format(
                    description, inputvalue, maxvalue))

def _check_bool(inputvalue, description="inputvalue"):
    """Check that the given inputvalue is a boolean"""

    _check_string(description, minlength=1, description="description string")
    if not isinstance(inputvalue, bool):
        raise TypeError(
            "The {0} must be boolean. Given: {1!r}".format(description, inputvalue))
# Python code transmits a byte to Arduino /Microcontroller

import serial
import sys
import time
from numpy.fft import fft
import struct


def init_serial_connection():
    serial_obj = 0
    try:
        serial_obj = serial.Serial('/dev/ttyACM0')  # ttyUSBx format on Linux
    except serial.serialutil.SerialException as error:
        print(error)
        print('exiting')
        exit(1)
    serial_obj.baudrate = 115200  # set Baud rate to 9600
    serial_obj.bytesize = 8  # Number of data bits = 8
    serial_obj.parity = 'N'  # No parity
    serial_obj.stopbits = 1  # Number of Stop bits = 1
    serial_obj.timeout = 30
    time.sleep(0.5)
    return serial_obj


if __name__ == '__main__':
    run_mode = {'range': 'r', 'speed': 's'}  # dictionary of running modes
    len_time_sec = 5  # time in seconds for range measurement
    delay_time_sec = 0.5

    command = str('mode:' + run_mode.get('range')) + '\n' + 'time:' + str(len_time_sec) + '\n'
    print('sending: ', end='')
    print(bytes(command, 'utf-8'))

    stm_serial_com = init_serial_connection()
    stm_serial_com.write(bytes(command, 'utf-8'))
    time_out = time.time() + len_time_sec + delay_time_sec
    while time.time() < time_out:
        data_return = stm_serial_com.read()
        
        num = int.from_bytes(data_return, "little")
        print(num)
    stm_serial_com.close()

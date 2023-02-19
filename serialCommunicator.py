# Python code transmits a byte to Arduino /Microcontroller

import serial
import sys
import time
from numpy.fft import fft
import struct
import os

output_file = 'output.txt'

def init_serial_connection():
    serial_obj = 0
    try:
        serial_obj = serial.Serial('COM5')  # ttyUSBx format on Linux / COMx format for Windows
    except serial.serialutil.SerialException as error:
        print(error)
        print('exiting')
        exit(1)
    serial_obj.baudrate = 115200  # set Baud rate to 115200
    serial_obj.bytesize = 8  # Number of data bits = 8
    serial_obj.parity = 'N'  # No parity
    serial_obj.stopbits = 1  # Number of Stop bits = 1
    serial_obj.timeout = 30
    time.sleep(1)
    return serial_obj

if __name__ == '__main__':
    run_mode = {'range': 'r', 'speed': 's'}  # dictionary of running modes
    len_time_sec = 5  # time in seconds for range measurement
    delay_time_sec = 0.5

    command = str('mode:' + run_mode.get('range')) + '\n' + 'time:' + str(len_time_sec) + '\n'
    print('sending: ', end='')
    print(bytes(command, encoding='latin-1'))

    with open(output_file, 'w') as f:
        stm_serial_com = init_serial_connection()
        stm_serial_com.write(bytes(command, encoding='latin-1'))
        time_out = time.time() + len_time_sec + delay_time_sec
        while time.time() < time_out:
            data_return = stm_serial_com.read(2)
            num = (data_return[1] << 8) + (data_return[0] << 0)
            print(num)
            f.write(str(num))
            f.write('\n')
        stm_serial_com.close()

print('Output saved to ' + os.path.abspath(output_file))

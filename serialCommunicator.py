# MappingRadar Control Script
# Made by William Ralston & Evan Stenger
# ECE 791/792

import numpy
import serial
import sys
import time
from numpy.fft import fft
import struct
import os
import matplotlib.pyplot as plt

output_file = 'output.txt'


def init_serial_connection():
    serial_obj = 0
    platform = sys.platform
    try:
        if platform == "win32":
            serial_obj = serial.Serial('COM5')  # ttyUSBx format on Linux / COMx format for Windows
        else:
            serial_obj = serial.Serial('/dev/ttyACM1')  # ttyUSBx format on Linux / COMx format for Windows
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
    data_set = []
    command = str('mode:' + run_mode.get('speed') + '\n' + 'time:' + str(len_time_sec) + '\n')

    with open(output_file, 'w') as f:
        stm_serial_com = init_serial_connection()
        print('sending: ', end='')
        print(command.encode(encoding="utf-8"))
        while stm_serial_com.in_waiting:
            stm_serial_com.read_all()

        stm_serial_com.write(command.encode(encoding="utf-8"))
        time_out = time.time() + len_time_sec + delay_time_sec
        while time.time() < time_out:
            data_return = stm_serial_com.read(2)
            num = (data_return[1] << 8) + (data_return[0] << 0)
            print(num)
            f.write(str(num))
            f.write('\n')
            data_set.append(num)
        stm_serial_com.close()

    print('Output saved to ' + os.path.abspath(output_file))

    data_set = numpy.array(data_set)*(3.3/65536)
    plt.figure(1)
    plt.plot(data_set)
    plt.title("data returned")
    plt.xlabel("sample")
    plt.ylabel("amplitude (V)")
    plt.ylim([0, 3.3])

    plt.figure(2)
    fft_data = fft(data_set)
    plt.plot(numpy.abs(fft_data))
    plt.show()

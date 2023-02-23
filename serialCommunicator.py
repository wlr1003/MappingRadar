# MappingRadar Control Script
# Made by William Ralston & Evan Stenger
# ECE 791/792
# signal generator at 2.455653 for test run in lab


import os
import sys
import time
import serial
import numpy as np
from numpy.fft import fft
import scipy.constants as sc
import matplotlib.pyplot as plt

############################################
# Set time and mode defaults
TIME_DEFAULT = 10
MODE_DEFAULT = 'range'
#  turn off connection to stm32, loads file as data to process
CONNECT_TO_STM = False
load_file = "outputCans2.txt"  # file to load to get data to process

SAMPLING_FREQUENCY = 40000  # radar sampling frequency
SAMPLING_BITS = 2 ** 16  # 16 bit samples
CTUNE_FREQUENCY = 2455650000  # 2.45 GHz measured on spectrum analyzer
MAX_RANGE_METERS = 100
MAX_SPEED_KMH = 200  # 200kmh ~ 124mph
output_file = 'outputCans7.txt'  # file name to create and save returned data


# initialize a serial connection with stm32
# returns a serial object or exits program on error
def init_serial_connection():
    serial_obj = 0
    platform = sys.platform  # get platform string, win32/linux
    try:
        if platform == "win32":
            serial_obj = serial.Serial('COM5')  # COMx format for Windows
        else:
            serial_obj = serial.Serial('/dev/ttyACM0')  # ttyACMx format on Linux
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


# removes data from end to truncate data set to last complete 100 ms block
def trim_data(input_data):
    num_samples = np.shape(input_data)[0]  # get total sample number
    num_remove = int(
        num_samples % (SAMPLING_FREQUENCY / 10))  # get remainder of total sample number and samples per 100 ms
    indexes_remove = np.array(
        [num_samples - i - 1 for i in range(num_remove)])  # create array of indexes to remove from data
    return np.delete(input_data, indexes_remove)  # return input data truncated to last full 100 ms block


if __name__ == '__main__':
    run_mode = {'range': 'r', 'speed': 's'}  # dictionary of running modes
    len_time_sec = TIME_DEFAULT  # time in seconds for range measurement
    delay_time_sec = 0.5  # delay added to ensure all samples transferred
    command = str('mode:' + run_mode.get(MODE_DEFAULT) + '\n' + 'time:' + str(len_time_sec) + '\n').encode(
        encoding="utf-8")  # assemble  and encode command to send to stm32

    if CONNECT_TO_STM:
        with open(output_file, 'w') as f:
            stm_serial_com = init_serial_connection()
            time_out = time.time() + len_time_sec + delay_time_sec  # calculate time to end data collection
            data_set = []  # initialize list for data returned from stm32
            print('sending: ', end='')
            print(command)
            while stm_serial_com.in_waiting:  # if data is in incoming buffer, read past all data
                stm_serial_com.read_all()

            stm_serial_com.write(command)  # send command to stm32
            while time.time() < time_out:  # read data returned until timeout
                data_return = stm_serial_com.read(2)  # read 2 bytes
                num = (data_return[1] << 8) + (data_return[0] << 0)  # assemble two bytes into 16-bit number
                print(num)
                f.write(str(num))  # save num to file
                f.write('\n')  # add delimiter to file
                data_set.append(num)  # append data to list of data returned
            stm_serial_com.close()

        print('Output saved to ' + os.path.abspath(output_file))
    else:
        print('Loading data from: ' + load_file)
        data_set = np.loadtxt(load_file, comments="#", delimiter="\n", unpack=False)

    data_set = np.array(data_set) * (3.3 / SAMPLING_BITS)  # scale to voltage
    data_set = trim_data(data_set)  # trim data to last complete 100ms block
    N_trimmed = data_set.size  # get number of samples kept
    N_padded = 65536  # padding used for increased resolution of fft
    total_time = N_trimmed / SAMPLING_FREQUENCY

    # plot of entire data set
    plt.figure(1)
    t = np.array([i / SAMPLING_FREQUENCY for i in range(N_trimmed)])  # set t from 0 to time of last sample
    plt.plot(t, data_set)
    plt.title("Data Returned scaled to Voltage Applied to ADC")
    plt.xlabel("time (sec)")
    plt.ylabel("amplitude (V)")
    plt.ylim([0, 3.3])

    # plot of fft magnitude of entire data set vs frequency
    plt.figure(2)
    fft_data = fft(data_set)
    # set f to 0 to sampling frequency for x-axis
    f = np.array([i * SAMPLING_FREQUENCY / (N_trimmed - 1) for i in range(N_trimmed)])
    plt.plot(f, np.abs(fft_data))
    plt.title("FFT of entire data set")
    plt.ylabel("Magnitude")
    plt.xlabel("Frequency (Hz)")

    # split data set into array with each row being 100 ms of samples
    data_split = np.array(np.split(data_set, int(N_trimmed / (SAMPLING_FREQUENCY / 10))))
    # remove DC offset from data by removing mean from each 100ms row
    mean = data_split.mean(axis=1)
    data_split = data_split - mean[:, None]
    # take fft of every row with zero padding
    fft_data = fft(data_split, N_padded)

    # plot of fft for single 100ms row, row 20 chosen at random
    plt.figure(3)
    # set f to 0 to sampling frequency for x-axis
    f = np.array([i * SAMPLING_FREQUENCY / (N_padded - 1) for i in range(N_padded)])
    # prevent errors if fft data has less than 2 seconds of data, take row 20 or last
    slice_num = min(20, np.shape(fft_data)[0])
    plt.plot(f, np.abs(fft_data[slice_num]))
    plt.title("FFT of time slice " + str(slice_num))
    plt.ylabel("Magnitude")
    plt.xlabel("Frequency (Hz)")

    speed_m_s = MAX_SPEED_KMH*1000/(60*60)  # convert km/h to m/s
    max_delt_freq = speed_m_s/sc.speed_of_light*CTUNE_FREQUENCY  # del_f = (Ve/c)*fc
    # keep fft data proportional to MAX_SPEED
    num_keep = int(N_padded / SAMPLING_FREQUENCY * max_delt_freq)
    fft_data = fft_data[:, :num_keep]

    plt.figure(4)
    range_data = np.transpose(np.abs(fft_data))
    aspect_ratio = np.shape(fft_data)[0] / np.shape(fft_data)[1] * (5/7)
    plt.imshow(range_data, origin='lower', aspect=aspect_ratio, extent=[0, total_time, 0, speed_m_s])
    plt.title("Speed of Target")
    plt.xlabel('Time (s)')
    plt.ylabel('Speed (m/s)')
    plt.show()

# MappingRadar Control Script
# Made by William Ralston & Evan Stenger
# ECE 791/792
# signal generator at 2.455653 for test run in lab


import os
import sys
import time
import serial
from scipy import signal
from scipy import constants
from scipy import interpolate
import numpy as np
from numpy.fft import fft
import matplotlib.pyplot as plt

############################################
# Set time and mode defaults
TIME_DEFAULT = 10
MODE_DEFAULT = 'speed'
#  turn off connection to stm32, loads file as data to process
CONNECT_TO_STM = False
load_file = "outputCans2.txt"  # file to load to get data to process

SAMPLING_FREQUENCY = 40000  # radar sampling frequency
SAMPLING_BITS = 2 ** 16  # 16 bit samples
CTUNE_FREQUENCY = 2455650000  # 2.45 GHz measured on spectrum analyzer
MAX_RANGE_METERS = 100
MAX_SPEED_KMH = 200  # 200kmh ~ 124mph
output_file = 'outputCans7.txt'  # file name to create and save returned data


class Signal_Processing_Control:
    def __init__(self):
        self.sift_stop_criteria = ['standard deviation', 0.025]  # only standard deviation implemented so far
        self.emd_stop_criteria = ['n times', 5]  # ['n times', 10] or None
        self.plot_imfs = True
        self.data_set = []
        self._figure_num = 0

# returns the current figure number used during plotting
    def fig_num(self):
        self._figure_num += 1
        return self._figure_num


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


# print the usage for the program to command line
def print_usage():
    print('Usage:')
    print(' serialCommunicator.py [options] [value]')
    print('Example:\n'
          ' serialCommunicator.py -m r -t 10\n')
    print('Options:')
    print('  -time, -t        Time for recording data from radar.\n'
          '  -mode, -m        Mode to set prior to recording data.\n'
          '  -help, -h        Print usage to console.')
    print('Value:')
    print('  -time, -t        Integer in seconds.\n'
          '  -mode, -m        Available modes: (r)ange, (s)peed.')


# parse all input arguments
# sets parameters passed as arguements or default
def parse_input_args():
    len_time = 0
    mode_select = ''
    num_args = len(sys.argv)
    if num_args > 1:
        for i in range(int((num_args - 1) / 2 + ((num_args - 1) % 2))):  # for each arg pair
            flag = sys.argv[(i * 2) + 1].strip().strip('-').lower()
            if flag == 't' or flag == 'time':
                try:
                    len_time = int(sys.argv[(i * 2) + 2])
                    if len_time < 0:
                        print('Invalid time: time must be integer greater than zero, given in seconds')
                        print_usage()
                        exit(2)
                except ValueError as error:
                    print('Invalid time: time must be integer greater than zero, given in seconds')
                    print_usage()
                    exit(2)
            elif flag == 'm' or flag == 'mode':
                mode_select = sys.argv[(i * 2) + 2].strip()
                if mode_select not in run_mode:
                    mode_set = False
                    for key in run_mode:
                        if run_mode.get(key) == mode_select:
                            mode_select = key
                            mode_set = True
                            break
                    if not mode_set:
                        print('Invalid mode: mode can be range or r, speed or s.')
                        print_usage()
                        exit(3)
            elif flag == 'h' or flag == 'help':
                print_usage()
                exit(0)
    return len_time, mode_select


def process_as_speed_data(data, speed):
    max_delt_freq = speed / constants.speed_of_light * CTUNE_FREQUENCY  # del_f = (Ve/c)*fc
    # keep fft data proportional to MAX_SPEED
    num_keep = int(N_padded / SAMPLING_FREQUENCY * max_delt_freq)
    data = data[:, :num_keep]
    return data


def emd(data_1d: np.ndarray, ctrl: Signal_Processing_Control):
    emd_stop_criteria_met = False
    intrinsic_mode_functions = []
    residual = np.copy(data_1d)  # residual is h_1
    current_residual = None     # variables for tracking residual during sifting h_1k
    last_residual = None        # h_1(k-1)
    x_axis = np.arange(0, len(residual) / SAMPLING_FREQUENCY, 1 / SAMPLING_FREQUENCY)  # generate an x-axis of time
    while not emd_stop_criteria_met:    # produce imf's until monotonic function remains or n imf's produced
        current_residual = np.copy(residual)  # reset current residual to the total residual
        sift_count = 0
        sift_stop_criteria_met = False
        while not sift_stop_criteria_met:  # sift data until imf is produced based on selected stop criteria
            sift_count += 1
            last_residual = np.copy(current_residual)  # get copy of h1(k-1) for testing sd
            # find maxima/minima of data set
            maxima = signal.argrelmax(current_residual)[0]  # returns a tuple of indexes
            minima = signal.argrelmin(current_residual)[0]
            if len(maxima) == 0 or len(minima) == 0:
                emd_stop_criteria_met = True
                break
            # interpolate the maxima/minima using a cubic spline
            upper_spline = interpolate.InterpolatedUnivariateSpline(maxima, current_residual[maxima], k=3)  # returns a spline class
            lower_spline = interpolate.InterpolatedUnivariateSpline(minima, current_residual[minima], k=3)
            # passing the x-axis as an argument returns the spline defined within the range of the x-axis
            upper_spline = upper_spline(range(len(x_axis)))
            lower_spline = lower_spline(range(len(x_axis)))
            spline_mean = np.mean([upper_spline, lower_spline], axis=0)  # find the mean of the spline envelope
            # plt.figure(10)
            # plt.plot(x_axis,current_residual,label='current residual pre')
            current_residual -= np.reshape(spline_mean,current_residual.shape)

            # plt.plot(x_axis,current_residual, label='current residual post')
            # plt.plot(x_axis,spline_mean, label='spline mean')
            # plt.plot(x_axis,upper_spline, label='upper spline')
            # plt.plot(x_axis,lower_spline, label='lower spline')
            # plt.legend()
            # plt.show()
            extrema_difference = abs(len(maxima)-len(minima))
            if ctrl.sift_stop_criteria[0] == 'standard deviation':
                if sift_count > 1 and extrema_difference <= 1:
                    st = np.max((maxima[0], minima[0]))  # check after both start
                    en = np.min((maxima[-1], minima[-1]))  # check before both end
                    stop_criteria = np.sum((last_residual[st:en] - current_residual[st:en]) ** 2) / np.sum(last_residual[st:en] ** 2)
                    if stop_criteria < ctrl.sift_stop_criteria[1]:
                        sift_stop_criteria_met = True
        # sifting is complete, record the current residual as the imf component
        intrinsic_mode_functions.append(current_residual)
        residual -= current_residual
        if ctrl.emd_stop_criteria and ctrl.emd_stop_criteria[0] == 'n times':
            if len(intrinsic_mode_functions) >= ctrl.emd_stop_criteria[1]:
                emd_stop_criteria_met = True
        else:  # stop on monotonic residual
            if is_monotonic(residual):
                emd_stop_criteria_met = True
    return np.array(intrinsic_mode_functions)


# return True if the array is monotonic, False otherwise
# monotonic is array that is strictly increasing or decreasing
def is_monotonic(residual: np.array):
    difference = np.diff(residual)
    return np.all(difference) <= 0 or np.all(difference) >= 0


def plot_imfs(imfs: np.array, ctrl):
    x_axis_time = np.arange(0, len(imfs[0][0]) / SAMPLING_FREQUENCY, 1 / SAMPLING_FREQUENCY)  # generate an x-axis of time
    num = 1
    for i in range(1):
        for imf in imfs[i]:
            plt.figure(ctrl.fig_num())
            plt.plot(x_axis_time, imf)
            plt.title('IMF ' + str(num))
            num += 1
            plt.xlabel('Time (s)')
            plt.ylabel('Amplitude')


def plot_speed_result(data, speed, ctrl):
    plt.figure(ctrl.fig_num())
    range_data = np.transpose(np.abs(data))
    aspect_ratio = np.shape(data)[0] / np.shape(data)[1] * (5 / 7)
    plt.imshow(range_data, origin='lower', aspect=aspect_ratio, extent=[0, total_time, 0, speed])
    plt.title("Speed of Target")
    plt.xlabel('Time (s)')
    plt.ylabel('Speed (m/s)')


if __name__ == '__main__':
    run_mode = {'range': 'r', 'speed': 's', 'map': 'm'}  # dictionary of running modes
    control = Signal_Processing_Control()
    # parse input args, set default values if no argument given
    len_time_sec, mode_selected = parse_input_args()
    if len_time_sec <= 0:
        len_time_sec = TIME_DEFAULT  # time in seconds for measurement
    if mode_selected == '':
        mode_selected = MODE_DEFAULT
    delay_time_sec = 0.5  # delay added to ensure all samples transferred
    command = str('mode:' + run_mode.get(mode_selected) + '\n' + 'time:' + str(len_time_sec) + '\n').encode(
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
    plt.figure(control.fig_num())
    # t = np.array([i / SAMPLING_FREQUENCY for i in range(N_trimmed)])  # set t from 0 to time of last sample
    t = np.arange(0, N_trimmed / SAMPLING_FREQUENCY, 1 / SAMPLING_FREQUENCY)
    plt.plot(t, data_set)
    plt.title("Data Returned scaled to Voltage Applied to ADC")
    plt.xlabel("time (sec)")
    plt.ylabel("amplitude (V)")
    plt.ylim([0, 3.3])

    # plot of fft magnitude of entire data set vs frequency
    plt.figure(control.fig_num())
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
    emd_data = []
    emd_break = 0
    for row in data_split:
        emd_data.append(emd(row, control))
        emd_break += 1
        if emd_break > 1:
            break
    if control.plot_imfs and len(emd_data) > 0:
        plot_imfs(emd_data, control)
    # plot of fft for single 100ms row, row 20 chosen at random
    plt.figure(control.fig_num())
    # set f to 0 to sampling frequency for x-axis
    f = np.array([i * SAMPLING_FREQUENCY / (N_padded - 1) for i in range(N_padded)])
    # prevent errors if fft data has less than 2 seconds of data, take row 20 or last
    slice_num = min(20, np.shape(fft_data)[0])
    plt.plot(f, np.abs(fft_data[slice_num]))
    plt.title("FFT of time slice " + str(slice_num))
    plt.ylabel("Magnitude")
    plt.xlabel("Frequency (Hz)")

    if mode_selected == 'speed':
        speed_m_s = MAX_SPEED_KMH * 1000 / (60 * 60)  # convert km/h to m/s
        fft_data = process_as_speed_data(fft_data, speed_m_s)
        plot_speed_result(fft_data, speed_m_s, control)

    plt.show()  # call only once for all plots

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
from scipy.signal import hilbert, decimate
import numpy as np
from numpy.fft import fft
import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap

############################################
# Set time and mode defaults
TIME_DEFAULT = 30
MODE_DEFAULT = 'speed'
DIGITAL_POT_DEFAULT = 0x7f  # default value to send to stm32

#  turn off connection to stm32, loads file as data to process
CONNECT_TO_STM = False
output_file = 'output/speed7f2-2.txt'  # file name to create and save returned data
load_file = "output/speed7f2-1.txt"  # file to load to get data to process
############################################
#  pot = 3f
#  yagi
# cookie 1 stationary
# cookie 2 walking away, grab cookie, come back
# cookie 3 holding cookie sheet, walk away and back
# planar
# cookie 4 walk away, run to
# cans
# cookie 5 walk away walk back
#  max pot
# cookie 6 max pot, walk away walk back
# yagi
#  cookie 7 walk away walk back
# cookie 8 walk away whole time
# speed 9 ran away, run back
#  range 10 walk away whole time
# 11 amp moved to receive
#  12 car driving toward
#  pot set to zero

# radar parameters
MAX_RANGE_METERS = 120  # max range will alter the max range that will be plotted
MAX_SPEED_KMH = 50  # 200kmh ~ 124mph
SAMPLING_FREQUENCY = 40000  # radar sampling frequency
SAMPLING_BITS = 2 ** 16  # 16 bit samples from ADC
CTUNE_FREQUENCY = 2455650000  # 2.45 GHz measured on spectrum analyzer
VTUNE_BANDWIDTH = 80000000  # Bandwidth of vtune signal from VCO
VTUNE_PERIOD = 1/25  # period of vtune set by DAC output in s


# control class used to alter signal processing scheme
class Signal_Processing_Control:
    def __init__(self):
        self.print_time = True  # turn on/off text output detailing current status of processing
        self.plot_preprocessed = True  # turn on/off plot of signal received over time
        self.plot_during_processing = False  # turn on/off plots of 3 sample blocks before and after ensemble mean
        self.plot_ranging_fft_at_5_sec = False  # turn on/off plot of individual FFT after range scaling
        self.notch_filter = False  # implements a notch filter removing 25Hz signal
        self.filter_and_down_sample = True  # decimate by a factor of 5, not necessary, but speeds up processing
        self.ensemble_mean = True  # turn on/off the removal of the ensemble mean
        self.scale_for_range_loss = True  # turn on/off scaling of FFT returns to compensate signal power loss/distance
        self.pulse_canceller = 3  # set pulse cancellation to 0, 2, or 3
        self.emd_analysis = False   # turn on/off the emd analysis
        self.sift_stop_criteria = ['standard deviation', 0.025]  # only standard deviation implemented so far
        self.emd_stop_criteria = ['n times', 5]  # ['n times', 10] or None
        self.plot_imfs = True   # turn on/off plots of recovered imf's, only used if emd_analysis is True
        self.data_set = np.array(0)  # initialize data set to empty numpy array
        self.N_trimmed = 0      # number of samples after truncating data to be evenly divisible
        self._figure_num = 0    # protected variable, used to keep track of figures

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


# removes data from end to truncate data set to last complete 40 ms block
def trim_data(input_data):
    num_samples = np.shape(input_data)[0]  # get total sample number
    num_remove = int(
        num_samples % (SAMPLING_FREQUENCY * VTUNE_PERIOD))  # get remainder of total sample number and samples per 40 ms
    if num_remove == 0:
        return input_data
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
          '  -help, -h        Print usage to console.\n'
          '  -pot,  -p        Digital potentiometer gain setting.')
    print('Value:')
    print('  -time, -t        Integer in seconds.\n'
          '  -mode, -m        Available modes: (r)ange, (s)peed.\n'
          '  -pot,  -p        Integer or Hex value in range (0,127) or (0x0,0x7f).')


# parse all input arguments
# sets parameters passed as arguments or default
def parse_input_args():
    len_time = 0
    mode_select = None
    digital_pot = None
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
            elif flag == 'p' or flag == 'pot':
                digital_pot = int(sys.argv[(i * 2) + 2], 0)
                if digital_pot < 0 or digital_pot > 127:
                    print('Invalid pot setting: digital pot must be set in range of 0 to 127. \n'
                          'Setting may be given as integer or as hex value, denote hex values with prefix 0x.')
                    print_usage()
                    exit(4)
            elif flag == 'h' or flag == 'help':
                print_usage()
                exit(0)
    # if any arg not set, set to default
    if len_time <= 0:
        len_time = TIME_DEFAULT
    if mode_select is None:
        mode_select = MODE_DEFAULT
    if digital_pot is None:
        digital_pot = DIGITAL_POT_DEFAULT
    return len_time, mode_select, digital_pot


# takes split fft data and keeps data below the frequency index of the max speed
def process_as_speed_data(data, speed):
    max_delt_freq = 2 * speed / constants.speed_of_light * CTUNE_FREQUENCY  # del_f = (Ve/c)*fc
    # keep fft data proportional to MAX_SPEED
    num_keep = get_max_freq_index(max_delt_freq)
    data = data[:, :num_keep]
    return data


# function processes fft data into a range vs time plot
def process_as_range_data(data, control: Signal_Processing_Control):
    # ranging fft used mostly to fine tune scaling factor
    range_scale_coefficient = 1.2
    if ctrl.plot_ranging_fft_at_5_sec and data.shape[0] > 125:
        plt.figure(control.fig_num())
        range_scale = np.arange(start=0, stop=MAX_RANGE_METERS, step=MAX_RANGE_METERS/len(data[0])) ** range_scale_coefficient
        single_block = np.abs(data[25*5]) * range_scale
        single_block -= np.min(single_block)
        single_block = 20 * np.log10(offset_zeros(single_block))
        single_block = single_block[:int(len(single_block)/2)]

        x_axis = np.arange(start=0, stop=SAMPLING_FREQUENCY/2, step=SAMPLING_FREQUENCY/2/len(single_block))
        plt.plot(x_axis, single_block)
        print('fft mean = ' + str(np.mean(single_block)))

    # assume all targets are stationary,
    # data is mixer signal which is difference between current transmit and time delayed return signals
    ramp_rate = VTUNE_BANDWIDTH / (VTUNE_PERIOD / 2)  # rate of change of vtune signal
    # range=(c*f_m)/(2*ramp_rate)  rearranged to solve for fm: fm_max=range_max*(2*ramp_rate/c)
    max_delt_freq = MAX_RANGE_METERS * 2 * ramp_rate / constants.speed_of_light
    # get the index of the fft data proportional to the max range
    num_keep = get_max_freq_index(max_delt_freq)
    # keep samples below max freq index
    data = np.abs(data[:, :num_keep])

    aspect_ratio = total_time / MAX_RANGE_METERS * (5/7)
    if ctrl.scale_for_range_loss:
        range_scale = np.arange(start=0, stop=MAX_RANGE_METERS, step=MAX_RANGE_METERS/num_keep) ** range_scale_coefficient
        data_scaled = np.transpose(data[:]*range_scale)
        data_scaled = 20 * np.log10(offset_zeros(data_scaled))
        data_scaled -= data_scaled.min()
        plt.figure(control.fig_num())
        plt.imshow(data_scaled, origin='lower', aspect=aspect_ratio, extent=[0, total_time, 0, MAX_RANGE_METERS], vmin=np.mean(data_scaled))
        if ctrl.pulse_canceller != 0:
            plt.title(f"Range of Target with Returns Scaled and {ctrl.pulse_canceller} Pulse Canceller")
        else:
            plt.title("Range of Target with Returns Scaled for Range Loss")
        plt.xlabel('Time (s)')
        plt.ylabel('Distance (m)')
    plt.figure(control.fig_num())
    # range_data = 20 * np.log10(np.transpose(np.abs(data)))
    range_data = np.transpose(data)

    # range_min = np.min(range_data)
    # range_max = np.max(range_data)
    # print(f'range min val = {range_min}, range max val = {range_max}')
    # colors = [(0, 0, 0), (0.8, 0, 0), (1, 1, 0)]  # first color is black, second is red, third is yellow
    # cmap = LinearSegmentedColormap.from_list("Custom", colors, N=256)
    # plt.imshow(range_data, origin='lower', cmap=cmap, aspect=aspect_ratio, extent=[0, total_time, 0, MAX_RANGE_METERS])
    plt.imshow(range_data, origin='lower', aspect=aspect_ratio, extent=[0, total_time, 0, MAX_RANGE_METERS])
    plt.title("Range of Target")
    plt.xlabel('Time (s)')
    plt.ylabel('Distance (m)')


# function returns the fft index of the maximum frequency
def get_max_freq_index(max_delt_freq):
    num_keep = int(N_padded / SAMPLING_FREQUENCY * max_delt_freq)
    return num_keep


# function adds small offset (0.001) to any zero values of the data passed
# data can be 1d or 2d array
# used before a conversion to dB, a value of 0.001 will equal -60dB
def offset_zeros(data):
    if len(data.shape) > 1:
        for x in data:
            offset_zeros(x)
        return data
    for n in range(data.shape[0]):
        if data[n] < 0.001:
            data[n] = 0.001
    return data


# function performs empirical mode decomposition analysis and returns array of intrinsic mode functions
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
            if len(maxima) <= 3 or len(minima) <= 3:
                emd_stop_criteria_met = True
                break
            # interpolate the maxima/minima using a cubic spline
            upper_spline = interpolate.InterpolatedUnivariateSpline(maxima, current_residual[maxima], k=3)  # returns a spline class
            lower_spline = interpolate.InterpolatedUnivariateSpline(minima, current_residual[minima], k=3)
            # passing the x-axis as an argument returns the spline defined within the range of the x-axis
            upper_spline = upper_spline(range(len(x_axis)))
            lower_spline = lower_spline(range(len(x_axis)))
            spline_mean = np.mean([upper_spline, lower_spline], axis=0)  # find the mean of the spline envelope
            # debug plot
            # plt.figure(10)
            # plt.plot(x_axis,current_residual,label='current residual pre')

            current_residual -= np.reshape(spline_mean, current_residual.shape)

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
        if ctrl.print_time:
            print(f'IMF {len(intrinsic_mode_functions)} found after sifting {sift_count} times.', end='')
            print_time_elapsed(time_start)
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


# function plots array of intrinsic mode functions
def plot_imfs(imfs: np.array,instantaneous_freq: np.array,  control: Signal_Processing_Control):
    x_axis_time = np.arange(0, len(imfs[0]) / SAMPLING_FREQUENCY, 1 / SAMPLING_FREQUENCY)  # generate an x-axis of time
    num = 1  # num is index of imf used for plot title
    avg = 3  # number of points to include in moving average

    for imf in imfs:
        plt.figure(control.fig_num())
        plt.subplot(211)
        plt.plot(x_axis_time, imf)
        plt.title('IMF ' + str(num))
        plt.xlabel('Time (s)')
        plt.ylabel('Amplitude')
        num += 1

        plt.subplot(212)
        ramp_rate = VTUNE_BANDWIDTH / (VTUNE_PERIOD / 2)  # rate of change of vtune signal
        delt_freq = 2 * ramp_rate / constants.speed_of_light
        instantaneous_freq[num-2] = np.abs(instantaneous_freq[num-2]) / delt_freq
        plt.plot(x_axis_time[0:len(instantaneous_freq[num-2])], instantaneous_freq[num-2])
        plt.title("Range of Target from Instantaneous Frequency of IMF")
        plt.xlabel('Time (s)')
        plt.ylabel('Target Range (m)')
        plt.ylim([0, MAX_RANGE_METERS])
        # label = str(avg) + ' point moving avg'
        # plt.plot(x_axis_time[0:len(instantaneous_freq[num-2])], np.convolve(instantaneous_freq[num-2], np.ones(avg)/avg, 'same'), label=label)
        # plt.plot(x_axis_time[0:len(instantaneous_freq[num-2])], np.convolve(instantaneous_freq[num-2], np.ones(5)/5, 'same'), label='5')
        avg = int(SAMPLING_FREQUENCY / 50)
        plt.plot(x_axis_time[0:len(instantaneous_freq[num-2])], np.convolve(instantaneous_freq[num-2], np.ones(avg)/avg, 'same'), label=str(avg))


# function plots the FFT results as speed returns
def plot_speed_result(data, speed, ctrl):
    plt.figure(ctrl.fig_num())
    range_data = np.transpose(np.abs(data))
    aspect_ratio = total_time / speed * (5/7)
    plt.imshow(range_data, origin='lower', aspect=aspect_ratio, extent=[0, total_time, 0, speed])
    plt.title("Speed of Target")
    plt.xlabel('Time (s)')
    plt.ylabel('Speed (m/s)')


# function plots the data set within the control struct over time
def plot_signal_over_time(control: Signal_Processing_Control):
    t = np.arange(0, control.N_trimmed / SAMPLING_FREQUENCY, 1 / SAMPLING_FREQUENCY)
    # plot of entire data set
    plt.figure(control.fig_num())
    plt.plot(t, control.data_set, '.')
    plt.title("Data Returned scaled to Voltage Applied to the ADC")
    plt.xlabel("Time (sec)")
    plt.ylabel("Amplitude (V)")
    plt.ylim([0, 3.3])


# function prints time elapsed from the passed prev_time
def print_time_elapsed(prev_time):
    print(' Time elapsed: ' + str(time.time()-prev_time))


if __name__ == '__main__':
    delay_time_sec = 0.5  # delay added to ensure all samples transferred
    N_padded = 2 ** 14  # padding used for increased resolution of fft
    run_mode = {'range': 'r', 'speed': 's', 'map': 'm', 'low': 'l', 'high': 'h'}  # dictionary of running modes
    ctrl = Signal_Processing_Control()
    time_start = time.time()
    # parse input args, set default values if no argument given
    len_time_sec, mode_selected, digital_pot_setting = parse_input_args()

    # assemble and encode command to send to stm32
    command = str('mode:' + run_mode.get(mode_selected) + '\n' +
                  'time:' + str(len_time_sec) + '\n' +
                  'pot:' + str(digital_pot_setting) + '\n').encode(encoding="utf-8")

    if CONNECT_TO_STM:
        with open(output_file, 'w') as f:
            stm_serial_com = init_serial_connection()
            time_out = time.time() + len_time_sec + delay_time_sec  # calculate time to end data collection
            ctrl.data_set = []  # initialize list for data returned from stm32
            print('sending: ', end='')
            print(command)
            while stm_serial_com.in_waiting:  # if data is in incoming buffer, read past all data
                stm_serial_com.read_all()

            stm_serial_com.write(command)  # send command to stm32
            while time.time() < time_out:  # read data returned until timeout
                if stm_serial_com.in_waiting:
                    data_return = stm_serial_com.read(2)  # read 2 bytes
                    num = (data_return[1] << 8) + (data_return[0] << 0)  # assemble two bytes into 16-bit number
                    print(num)
                    f.write(str(num))  # save num to file
                    f.write('\n')  # add delimiter to file
                    ctrl.data_set.append(num)  # append data to list of data returned
            stm_serial_com.close()
        print('Output saved to ' + os.path.abspath(output_file))
    else:
        print('Loading data from: ' + load_file)
        ctrl.data_set = np.loadtxt(load_file, comments="#", delimiter="\n", unpack=False)
        if ctrl.data_set.size == 0:
            exit(0)
    if ctrl.print_time:
        print(f'Data acquisition complete. {len(ctrl.data_set)} samples.', end='')
        print_time_elapsed(time_start)

    print('Begin Processing')
    ctrl.data_set = np.array(ctrl.data_set) * (3.3 / SAMPLING_BITS)  # scale to voltage
    ctrl.data_set = trim_data(ctrl.data_set)  # trim data to last complete block size
    ctrl.N_trimmed = ctrl.data_set.size  # get number of samples kept
    total_time = ctrl.N_trimmed / SAMPLING_FREQUENCY

    # plot of signal in time domain scaled to input voltage of ADC
    if ctrl.plot_preprocessed:
        plot_signal_over_time(ctrl)

    # remove DC offset
    ctrl.data_set = ctrl.data_set-np.mean(ctrl.data_set)
    if ctrl.print_time:
        print('DC offset removed.', end='')
        print_time_elapsed(time_start)
    if ctrl.plot_preprocessed:
        # plot of fft magnitude of entire data set vs frequency
        plt.figure(ctrl.fig_num())
        fft_data = fft(ctrl.data_set)
        # set f to 0 to sampling frequency for x-axis
        f = np.array([i * SAMPLING_FREQUENCY / (ctrl.N_trimmed - 1) for i in range(ctrl.N_trimmed)])
        plt.yscale('log')
        plt.plot(f, np.abs(fft_data))
        plt.title("FFT of entire data set")
        plt.ylabel("Magnitude")
        plt.xlabel("Frequency (Hz)")
        if ctrl.print_time:
            print('Preprocessed plots complete.', end='')
            print_time_elapsed(time_start)

    if ctrl.notch_filter:
        f0 = 25.0  # Frequency to be removed from signal (Hz)
        Q = 30.0  # Quality factor
        # Design notch filter
        b, a = signal.iirnotch(f0, Q, SAMPLING_FREQUENCY)
        ctrl.data_set = signal.filtfilt(b, a, ctrl.data_set)
        ctrl.data_set = signal.filtfilt(b, a, ctrl.data_set)
        # plot resulting fft of data set
        plt.figure(ctrl.fig_num())
        fft_data = fft(ctrl.data_set)
        # set f to 0 to sampling frequency for x-axis
        f = np.array([i * SAMPLING_FREQUENCY / (ctrl.N_trimmed - 1) for i in range(ctrl.N_trimmed)])
        plt.yscale('log')
        plt.plot(f, np.abs(fft_data))
        plt.title("FFT of entire data set after notch filter")
        plt.ylabel("Magnitude")
        plt.xlabel("Frequency (Hz)")
        if ctrl.print_time:
            print('Notch filtering complete.', end='')
            print_time_elapsed(time_start)

    if ctrl.filter_and_down_sample:
        down_sample_factor = 5  # set factor to down sample by, original = 40 kHz, (note, 100m range = 3.3kHz signal)
        # decimate uses a 20*down_sample_factor order fir filter with a Hamming window for anti aliasing,
        # filter is applied forward then backward to negate phase shift
        ctrl.data_set = decimate(ctrl.data_set, q=down_sample_factor, ftype='fir', zero_phase=True)
        # set variables that changed due to decimation
        SAMPLING_FREQUENCY = SAMPLING_FREQUENCY / down_sample_factor
        ctrl.N_trimmed = ctrl.data_set.size
        if ctrl.print_time:
            print(f'Down sampling by {down_sample_factor} complete.', end='')
            print_time_elapsed(time_start)

    # split data set into array with each row being 40 ms of samples
    data_split = np.array(np.split(ctrl.data_set, int(ctrl.N_trimmed / (SAMPLING_FREQUENCY * VTUNE_PERIOD))))
    if ctrl.print_time:
        print(f'Data split complete. {len(data_split[0])} samples per segment. ', end='')
        print_time_elapsed(time_start)
    if ctrl.ensemble_mean:
        if ctrl.plot_during_processing:
            plt.figure(ctrl.fig_num())
            plt.plot(data_split[0])
            plt.title('first sample before ensemble mean removal')
            plt.figure(ctrl.fig_num())
            plt.plot(data_split[9])
            plt.title('tenth sample before ensemble mean removal')
            plt.figure(ctrl.fig_num())
            plt.plot(data_split[-1])
            plt.title('last sample before ensemble mean removal')
        # remove ensemble mean from data by removing mean from each column
        mean = data_split.mean(axis=0)
        data_split = data_split - mean
        if ctrl.plot_during_processing:
            plt.figure(ctrl.fig_num())
            plt.plot(data_split[0])
            plt.title('first sample after ensemble mean removal')
            plt.figure(ctrl.fig_num())
            plt.plot(data_split[9])
            plt.title('tenth sample after ensemble mean removal')
            plt.figure(ctrl.fig_num())
            plt.plot(data_split[-1])
            plt.title('last sample after ensemble mean removal')
        if ctrl.print_time:
            print('Ensemble mean removed.', end='')
            print_time_elapsed(time_start)

    if ctrl.pulse_canceller == 2:
        data_split = data_split[2:] - data_split[1:-1]
        if ctrl.print_time:
            print('Two pulse cancellation complete.', end='')
            print_time_elapsed(time_start)
    elif ctrl.pulse_canceller == 3:
        data_split = data_split[3:] - 2*data_split[2:-1] + data_split[1:-2]
        if ctrl.print_time:
            print('Three pulse cancellation complete.', end='')
            print_time_elapsed(time_start)

    # take fft of every row with zero padding
    fft_data = fft(data_split, N_padded)
    if ctrl.print_time:
        print('FFT of signal complete.', end='')
        print_time_elapsed(time_start)
    if ctrl.emd_analysis:
        intrinsic_mode_functions = emd(ctrl.data_set, ctrl)
        if ctrl.print_time:
            print('EMD complete.', end='')
            print_time_elapsed(time_start)
        analytic_signal = hilbert(intrinsic_mode_functions)
        instantaneous_phase = np.unwrap(np.angle(analytic_signal))
        instantaneous_frequency = (np.diff(instantaneous_phase) / (2.0*np.pi) * SAMPLING_FREQUENCY)
        if ctrl.print_time:
            print('Instantaneous frequency calculated.', end='')
            print_time_elapsed(time_start)
        if ctrl.plot_imfs and len(intrinsic_mode_functions) > 0:
            plot_imfs(intrinsic_mode_functions, instantaneous_frequency, ctrl)

    if mode_selected == 'speed':
        speed_m_s = MAX_SPEED_KMH * 1000 / (60 * 60)  # convert km/h to m/s
        fft_data = process_as_speed_data(fft_data, speed_m_s)
        plot_speed_result(fft_data, speed_m_s, ctrl)
    elif mode_selected == 'range':
        process_as_range_data(fft_data, ctrl)

    plt.show()  # call only once for all plots

    if ctrl.print_time:
        print('Processing complete.', end='')
        print_time_elapsed(time_start)

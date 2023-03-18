clear all;
close all;

bandwidth = 80000000; % Bandwhith of transmit signal (hz)
period = .040; % Period of a full up-down ramp (s)
subdivid = 10; % Subdivid seconds (around 10 for best results)

ramprate = bandwidth/((1/2)*(period)); % (hz / s)
c = physconst('LightSpeed'); % Speed of light

freqRange = (20/c)*ramprate; % Hz per 10 meters (10 range = 20 meters range)
maxRange = 10 * freqRange; % maximum displayed range based on freqRange 10 meter incrememnts (default -> 10 * 10 = 100)

%[y, Fs] = audioread('running_outside_20ms.wav');
%data_channel = 2; % 1 for L, 2 for R (Refer to waveform in Audacity)

y = readmatrix('outputPlanar.txt');
data_channel = 1; % 1 for L, 2 for R (Refer to waveform in Audacity)
Fs = 40000;

y = (y - 32768) / 65536; % removing the assumed average (DC) (1/2 point of 16 bit unsigned int)

b = mod(length(y),Fs);
y = y(1:length(y)-b,data_channel); % truncate the data to last whole second

data_duration = length(y)/Fs; % Detect length of audio recording based on sample rate and total numeber of samples

timeC= data_duration * subdivid; %Time Constant for Slicing
L = (Fs/timeC); % Length of each Slice

imf = emd(y);
y = imf(:,4);

y = reshape(y, L*data_duration, timeC); % Segmenting y into time-based slices

Y = fft(y); % Performing FFT on each slice

P2 = abs(Y/L);
P1 = P2(1:L/2+1);
P1(2:end-1) = 2*P1(2:end-1);
f = Fs*(0:(L/2))/L;
p = plot(f,P1);
title("Single-Sided Amplitude Spectrum of Y")
xlabel("f (Hz)")
ylabel("|P1(f)|")
pause()

Y=Y(1:maxRange/subdivid,1:data_duration*subdivid); % truncate the data to max range (in frequency)

fY = flip(Y, 1); % flips on Y axis so close range is at the bottom

i = image(abs(fY), 'CDataMapping','scaled');

yticks([ 0, ...
    freqRange/subdivid, ...
    (2* freqRange)/subdivid, ...
    (3* freqRange)/subdivid, ...
    (4* freqRange)/subdivid, ...
    (5* freqRange)/subdivid, ...
    (6* freqRange)/subdivid, ...
    (7* freqRange)/subdivid, ...
    (8* freqRange)/subdivid, ...
    (9* freqRange)/subdivid, ...
    (10* freqRange)/subdivid]);
yticklabels([100,90,80,70,60,50,40,30,20,10,0]);

xticks([0,(5*subdivid), ...
    (10*subdivid), ...
    (15*subdivid), ...
    (20*subdivid), ...
    (25*subdivid), ...
    (30*subdivid), ...
    (35*subdivid), ...
    (40*subdivid), ...
    (45*subdivid), ...    
    (50*subdivid), ...
    (55*subdivid), ...
    (60*subdivid), ...
    (65*subdivid), ...
    (70*subdivid), ...
    (75*subdivid), ...
    (80*subdivid), ...
    (85*subdivid), ...
    (90*subdivid), ...
    (95*subdivid), ...
    (100*subdivid), ...    
    ]);
xticklabels([0,5,10,15,20,25,30,35,40,45,50,55,60,65,70,75,80,85,90,95,100]);

ylabel("Range (m)")
xlabel("Time")

colorbar
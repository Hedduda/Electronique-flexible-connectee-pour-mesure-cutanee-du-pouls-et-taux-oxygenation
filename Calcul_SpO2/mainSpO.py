import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from scipy.signal import find_peaks

def data_file(file, N):
    df = pd.read_csv(file, header=None, names=['Time', 'IR_data', 'RED_data'])
    # Apply the filter to select only Time data between 100 and 110
    df = df.query('100 <= Time <= 110')
    IR_data_array = np.array(df['IR_data'])
    Red_data_array = np.array(df['RED_data'])
    Temp_array = np.array(df['Time'])
    runningIR = running_mean(IR_data_array, N)  #using the mesure
    runningRED = running_mean(Red_data_array, N)
    smoothIRtime = Temp_array[(N-1):]  # get rid of the first few data points that are outside the smoothing window
    smoothREDtime = Temp_array[(N-1):]
    return IR_data_array, Red_data_array, Temp_array, runningIR, runningRED, smoothIRtime, smoothREDtime, df

def running_mean(x, N): #moving average
    cumsum = np.cumsum(np.insert(x, 0, 0))
    return (cumsum[N:] - cumsum[:-N]) / float(N) # N = It defines the number of data points that are taken into account in each average calculation

# Transformations and calculations (IR_calculation and Red_calculation)
def IR_calculation(IR_data_DC, time, sm, s_time):
    # Calculate the peaks and troughs for the data with and without the average
    troughsX = []; troughsY = []
    peaks, _ = find_peaks(IR_data_DC, distance=40) #distance between neighboring peaks
    peaksX = []; peaksY = []
    for i in range(len(peaks)):
        peaksY.append(IR_data_DC[peaks[i]])
        peaksX.append(time[peaks[i]])
        axs[0].scatter(peaksX[i], peaksY[i], color='yellow')
    troughdata = [-x for x in IR_data_DC]
    troughs, _ = find_peaks(troughdata, distance=40) #distance between neighboring peaks
    peaks_new, _ = find_peaks(sm, distance=40) #distance between neighboring peaks
    peaks_sX = []; peaks_sY = []
    for i in range(len(peaks_new)):
        peaks_sY.append(sm[peaks_new[i]])
        peaks_sX.append(s_time[peaks_new[i]])
        axs[0].scatter(peaks_sX[i], peaks_sY[i], color='pink')
    troughs_new, _ = find_peaks([-x for x in sm], distance=40) #distance between neighboring peaks
    troughsX_new = []; troughsY_new = []
    for i in range(len(troughs_new)):
        troughsX_new.append(s_time[troughs_new[i]])
        troughsY_new.append(sm[troughs_new[i]])
        axs[0].scatter(troughsX_new[i], troughsY_new[i], color='blue')
    return troughsX_new, troughsY_new, peaks_sY, peaks_sX, peaksX

def Red_calculation(Red_data_DC, time, sm_data, sm_time):
    # Calculate the peaks and troughs for the data with and without the average
    troughsX = []; troughsY = []
    peaks, _ = find_peaks(Red_data_DC, distance=40) #distance between neighboring peaks
    peaksX = []; peaksY = []
    for i in range(len(peaks)):
        peaksY.append(Red_data_DC[peaks[i]])
        peaksX.append(time[peaks[i]])
        axs[1].scatter(peaksX[i], peaksY[i], color='yellow', label='Peaks')
    troughdata = [-x for x in Red_data_DC]
    troughs, _ = find_peaks(troughdata, distance=40) #distance between neighboring peaks
    peaksX_s = []; peaksY_s = []
    peaks_new, _ = find_peaks(sm_data, distance=40) #distance between neighboring peaks
    for i in range(len(peaks_new)):
        peaksY_s.append(sm_data[peaks_new[i]])
        peaksX_s.append(sm_time[peaks_new[i]])
        axs[1].scatter(peaksX_s[i], peaksY_s[i], color='pink', label='Peaks')
    troughs_new, _ = find_peaks([-x for x in sm_data], distance=40) #distance between neighboring peaks
    troughsX_new = []; troughsY_new = []
    for i in range(len(troughs_new)):
        troughsX_new.append(sm_time[troughs_new[i]])
        troughsY_new.append(sm_data[troughs_new[i]])
        axs[1].scatter(troughsX_new[i], troughsY_new[i], color='blue', label='Local Minima')
    return troughsX_new, troughsY_new, peaksX_s, peaksY_s, peaksX

def calc_RoR(REDpeaks, REDtroughs, IRpeaks, IRtroughs):
    NoPulses = min(len(IRpeaks), len(IRtroughs), len(REDpeaks), len(REDtroughs))
    Rs = []
    for i in range(NoPulses):
        R = ((REDpeaks[i] - REDtroughs[i]) / ((REDtroughs[i] + REDpeaks[i]) / 2)) / ((IRpeaks[i] - IRtroughs[i]) / ((IRtroughs[i] + IRpeaks[i]) / 2))
        if R > 0: Rs.append(R)
    RoR = np.average(Rs)
    return RoR

def BPM(p_X_IR, p_X_Red):
    delta_IR = [p_X_IR[i + 1] - p_X_IR[i] for i in range(len(p_X_IR) - 1)]
    delta_RED = [p_X_Red[i + 1] - p_X_Red[i] for i in range(len(p_X_Red) - 1)]

    bpm_rate_IR = 60 / (sum(delta_IR) / len(delta_IR))
    bpm_rate_RED = 60 / (sum(delta_RED) / len(delta_RED))
    return bpm_rate_IR, bpm_rate_RED

# Replace 'file' with the actual path of your .csv file
file = '2024-06-12_09-21-11.csv'
IR_data_array, Red_data_array, Temp_array, runningIR, runningRED, smoothIRtime, smoothREDtime, df = data_file(file, N=30)

fig, axs = plt.subplots(2, 1, figsize=(12, 6))

# Executing calculations
tX_IR, tY_IR, pY_IR, pX_IR, p_X_IR = IR_calculation(IR_data_array, Temp_array, runningIR, smoothIRtime)
tX_RED, tY_RED, pX_RED, pY_RED, p_X_Red = Red_calculation(Red_data_array, Temp_array, runningRED, smoothREDtime)
calc = calc_RoR(pY_RED, tY_RED, pY_IR, tY_IR)

bpm_rate_RED, bpm_rate_IR = BPM(p_X_IR, p_X_IR)

print('Result:', calc)
SpO2 = 110 - 25 * calc   # Using the Texas Instrument equation

print('SpO2', SpO2)
print('BPM_IR:', bpm_rate_IR)
print('BPM_RED', bpm_rate_RED)

# Plotting the graphs
axs[0].plot(df['Time'], df['IR_data'])
axs[0].plot(smoothIRtime, runningIR, label='Smoothed IR Data', color='black')
axs[0].set_title('IR PPG Signal  \n  \n BPM: %.2f' % bpm_rate_IR)
axs[0].set_ylabel('IR Voltage (V)')
axs[0].set_xlabel('Time (s)')

axs[1].plot(df['Time'], df['RED_data'], color='red')
axs[1].plot(smoothREDtime, runningRED, label='Smoothed RED Data', color='black')
axs[1].set_title('RED PPG Signal \n \n BPM: %.2f' % bpm_rate_RED)
axs[1].set_ylabel('RED Voltage (V)')
axs[1].set_xlabel('Time (s) \n \n Rate SPO2: %.2f %%' % SpO2)
plt.tight_layout(h_pad=2)
plt.show()

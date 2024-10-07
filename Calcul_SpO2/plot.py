import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import numpy.fft
import scipy.fftpack
from scipy import sparse
from scipy.sparse.linalg import spsolve


#Function to open the file
def data_file(caminho_arquivo):
    df = pd.read_csv(caminho_arquivo, header=None, names=['Time', 'IR_data','RED_data'])
    IR_data_array = np.array(df['IR_data'])
    Red_data_array = np.array(df['RED_data'])
    Temp_array = np.array(df['Time'])
    return IR_data_array,Red_data_array,Temp_array,df



file = '2024-06-12_09-28-08.csv' #Add file path

IR_data_array,Red_data_array,Temp_array,df = data_file(file)


fig, axs = plt.subplots(2, 1, figsize=(12,6))

axs[0].plot(df['Time'], df['IR_data'])
axs[0].plot(Temp_array, IR_data_array, label='Smoothed IR Data', color='blue')
axs[0].set_title('IR PPG Signal')
axs[0].set_ylabel('IR Voltage (V)')
axs[0].set_xlabel('Time (s)')


axs[1].plot(df['Time'], df['RED_data'])
axs[1].set_title('RED PPG Signal')
axs[1].plot(Temp_array, Red_data_array, color='orange')
axs[1].set_ylabel('RED Voltage (V)')
axs[1].set_xlabel('Time (s)')



plt.tight_layout(h_pad=3)
plt.show()

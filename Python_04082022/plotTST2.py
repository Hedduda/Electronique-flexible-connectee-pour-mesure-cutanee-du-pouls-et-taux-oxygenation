# https://www.youtube.com/watch?v=zH0MGNJbenc
# Arduino with Python LESSON 11: Graphing and Plotting Live Data from Arduino with Matplotlib

import serial
import matplotlib.pyplot as plt 
import numpy as np 
from drawnow import *

ser = serial.Serial('COM4', 38400)
IRarray = []
plt.ion() 
cnt = 0

def makeFig():
	# plt.ylim(x,y)
	plt.title("STM32-AFE4490 PPG Signal")
	plt.grid(True)
	plt.ylabel("AFE4490 ADC Output")
	plt.plot(IRarray, 'r', label = 'IR Signal')
	plt.legend(loc='upper left')

while 1:
	while(ser.inWaiting() == 0):
		pass
	stm32_str = ser.readline().decode('ascii')
	IR = int(stm32_str.split(':')[1].split('\r')[0])
	# print(IR)
	IRarray.append(IR)
	drawnow(makeFig)
	plt.pause(0.000001)
	cnt = cnt + 1
	if (cnt > 500):
		IRarray.pop(0)


# Conclusion !
# Matplotlib is not suitable for our application !
# Our Data update every 2ms [f=500 Hz], so Matplotlib
# can't plot according to this speed frequency !
# We should to find another library/solution for 
# real time plotting 

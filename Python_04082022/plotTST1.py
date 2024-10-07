import matplotlib.pyplot as plt 
import serial

ser = serial.Serial('COM4', 38400)
# fig = plt.figure(figsize=(7, 3))
# ax = fig.add_subplot()

# fig.show()
# x=[]
# l=0
# l=1500
# ser.close()
# ser.open()

# for i in range(l):
# 	ser1 = ser.readline().decode('ascii')
# 	# ser2 = int(ser1)
# 	b = ser1.split()
# 	ser2 = int(b[1])

# 	x.append(ser2)
# 	ax.plot(x, color='b')
# 	fig.canvas.draw()
# 	ax.set_xlim(left=max(0, i-30), right=i+60)
# 	plt.pause(0.000001)

# plt.show()


# while 1:
# 	i=0
# 	i=i+1
# 	ser1 = ser.readline().decode('ascii')
# 	#ser2 = int(ser1)
# 	b = ser1.split()
# 	ser2 = int(b[1])

# 	x.append(ser2)
# 	ax.plot(x, color = 'b')
# 	fig.canvas.draw()
# 	ax.set_xlim(left=max(0, i-30), right=i+60)
# 	plt.pause(0.0001)


# plt.show()

while 1:
	ser1 = ser.readline().decode('ascii')
	print(ser1)

	
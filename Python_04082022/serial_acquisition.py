import serial


ser = serial.Serial("COM4", 38400)

while 1:
    IR = ser.readline().decode("ascii")
    print(IR)
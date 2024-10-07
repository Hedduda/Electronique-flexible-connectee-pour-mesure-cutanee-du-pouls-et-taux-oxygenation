import serial

ser = serial.Serial("COM3", 115200)

while 1:
    print(ser.readline())
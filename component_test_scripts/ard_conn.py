import serial
import time

baud_rate = 115200
arduino_port = ""
ser = serial.Serial(arduino_port, baud_rate, timeout=1)
time.sleep(2)
print(ser)
print("Arduino Connection Established!!")
import serial
import time
import csv

BAUD = 9600


ser = serial.Serial('/dev/ttyUSB1',BAUD)
ser.flushInput()

while True:
    try:
        ser_bytes = ser.readline()
        decoded_bytes = float(ser_bytes[0:len(ser_bytes)-2].decode("utf-8"))
        print(decoded_bytes)



        
    except:
        print("Keyboard Interrupt")
        break
        
        


import serial
import matplotlib.pyplot as plt
import time

ser = serial.Serial('/dev/cu.usbserial-8340', 9600)

pitch = []
roll = []
filtered_pitch = []
filtered_roll = []

while True:
    try:
        line = ser.readline().decode('UTF-8').strip()
        values = line.strip()
        if len(values) == 4:
            pitch, roll, filtered_pitch, filtered_roll = map(float, values)
        else:
            print("bad data")
    except ValueError:
        continue
    print(pitch, roll, filtered_pitch, filtered_roll)
    time.sleep (0.1)
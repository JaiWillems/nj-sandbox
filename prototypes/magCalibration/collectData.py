import serial
import time
import csv

port = '/dev/cu.usbserial-8340'
output_file = 'output.csv'

ser = serial.Serial(port,115200)

with open(output_file, 'w') as csvfile:
    writer = csv.writer(csvfile)
    writer.writerow(['time', 'x', 'y'])

    print ("Starting data collection...")
    while True:
        line = ser.readline().decode('utf-8')
        print("Reading data...")
        data = line.split(',')
        mag_x = data[0]
        mag_y = data[1]
        print (time.time())
        writer.writerow([time.time(), mag_x, mag_y])
        time.sleep(0.1)

ser.close()
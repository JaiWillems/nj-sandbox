import serial
import sqlite3
import re
import time

device_address = '/dev/cu.usbserial-8340'
baudrate = 9600

ser = serial.Serial(device_address, baudrate)
database = sqlite3.connect('prototypes/ultrasoundSensor.db')
cursor = database.cursor()
try:
    cursor.execute("SELECT 1 FROM ultrasoundSensor LIMIT 1")
except sqlite3.OperationalError:
    cursor.execute(
        "CREATE TABLE ultrasoundSensor(Time FLOAT, AbsoluteDistance FLOAT, RelativeDistance FLOAT)")
start_time = time.time()
while True:
    line = ser.readline().decode('utf-8').strip()
    numerical_data = re.sub('\t', '|', line)
    numerical_data = re.sub('[^0-9.|-]+', '', numerical_data)
    absolute_distance = numerical_data.split('|')[0]
    relative_distance = numerical_data.split('|')[1]
    timelapsed = time.time() - start_time
    cursor.execute(
        "INSERT INTO ultrasoundSensor(Time,AbsoluteDistance, RelativeDistance) VALUES(?,?,?)", (
            timelapsed, absolute_distance, relative_distance)
    )
    database.commit()

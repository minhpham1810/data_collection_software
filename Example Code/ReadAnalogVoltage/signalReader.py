import serial
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import csv
import time

# Constants
SERIAL_PORT = 'COM5'
BAUD_RATE = 9600

# Initialize serial connection
ser = serial.Serial(SERIAL_PORT, BAUD_RATE)

# Initialize start time to track recording time
startTime = time.time()

# Initialize empty lists to store data
x_vals = []
sensorValue1_data = []
sensorValue2_data = []

# Create a function to read and process data from Arduino
def read_and_process_data():
    line = ser.readline().decode('utf-8').strip()
    sensorValues = line.split(' ')
    x_vals.append(time.time() - startTime)
    sensorValue1_data.append(sensorValues[0])
    sensorValue2_data.append(sensorValues[1])

    # Print the received values
    print(f'Time : {x_vals[0]} Sensor 1: {sensorValues[0]}, Sensor 2: {sensorValues[1]}')

# Create a function to update the plot
def update_plot(frame):
    read_and_process_data()
    plt.cla()
    plt.plot(x_vals, sensorValue1_data, label='Sensor 1')
    plt.plot(x_vals, sensorValue2_data, label='Sensor 2')
    plt.xlabel('Time')
    plt.ylabel('Sensor Values')
    plt.legend()

# Create a function to save data to a CSV file when the plot window is closed
def on_close(event):
    with open('arduino_data.csv', 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(['Time', 'Sensor1', 'Sensor2'])
        for x, s1, s2 in zip(x_vals, sensorValue1_data, sensorValue2_data):
            writer.writerow([x, s1, s2])

# Register the callback function for when the plot window is closed
fig, ax = plt.subplots()
fig.canvas.mpl_connect('close_event', on_close)

ani = FuncAnimation(fig, update_plot, interval=1)
plt.show()
import csv

# Open the CSV file to read and load its content
with open("E:/Myoware Muscle Sensor/MyoWare_MuscleSensor/Example Code/ReadAnalogVoltage/arduino_data.csv", mode='r') as file:
    reader = csv.reader(file)
    rows = list(reader)  # Convert reader object to list for manipulation

# Modify the rows by filtering or deleting specific rows
# For example, deleting rows where the first column has 'delete_me' value
updated_rows = [row for row in rows if row[0] != 'delete_me']

# Open the CSV file again to write the updated data
with open("E:/Myoware Muscle Sensor/MyoWare_MuscleSensor/Example Code/ReadAnalogVoltage/arduino_data.csv", mode='w', newline='') as file:
    writer = csv.writer(file)
    writer.writerows(updated_rows)

print("Data updated successfully!")

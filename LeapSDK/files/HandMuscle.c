#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#ifdef _WIN32
#include <windows.h>
#else
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#endif

#define NUM_CHANNELS 8
#define ADC_MAX 1023.0
#define VREF 5.0
#define ADC_TO_MILLIVOLTS(x) (((x) / ADC_MAX) * (VREF * 1000.0))

// Global variables
double sensorData[NUM_CHANNELS]; // Holds the latest sensor values
FILE *csvFile;                   // File pointer

#ifdef _WIN32
HANDLE serial_port;
#else
int serial_port;
#endif

// Function to set up serial communication
int setup_serial_connection()
{
    char portName[256];

#ifdef _WIN32
    sprintf(portName, "\\\\.\\COM5"); // Change to the correct COM port
    serial_port = CreateFile(portName, GENERIC_READ, 0, NULL, OPEN_EXISTING, 0, NULL);
    if (serial_port == INVALID_HANDLE_VALUE)
    {
        fprintf(stderr, "Error opening port %s.\n", portName);
        return -1;
    }
#else
    sprintf(portName, "/dev/ttyUSB0"); // Change if needed
    serial_port = open(portName, O_RDWR | O_NOCTTY);
    if (serial_port < 0)
    {
        perror("Error opening serial port");
        return -1;
    }

    struct termios tty;
    tcgetattr(serial_port, &tty);
    cfsetispeed(&tty, B9600);
    cfsetospeed(&tty, B9600);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;
    tcsetattr(serial_port, TCSANOW, &tty);
#endif

    printf("Connected to Arduino on port: %s\n", portName);
    return 0;
}

// Function to read and process sensor data from Serial
int read_and_process_data(clock_t startTime)
{
    char line[256]; // Buffer for reading ASCII data
    DWORD bytesRead = 0;

#ifdef _WIN32
    if (!ReadFile(serial_port, line, sizeof(line) - 1, &bytesRead, NULL))
    {
        return -1;
    }
#else
    int bytesRead = read(serial_port, line, sizeof(line) - 1);
    if (bytesRead <= 0)
        return -1;
#endif

    line[bytesRead] = '\0'; // Null-terminate for safety

    // Tokenize and extract numbers
    char *token = strtok(line, ",");
    double sensorValues[NUM_CHANNELS] = {0}; // Default all to 0
    int count = 0;

    while (token && count < NUM_CHANNELS)
    {
        sensorValues[count] = atof(token);
        token = strtok(NULL, ",");
        count++;
    }

    double timestamp = (double)(clock() - startTime) / CLOCKS_PER_SEC;

    // Write to CSV
    fprintf(csvFile, "%.2f", timestamp);
    for (int i = 0; i < NUM_CHANNELS; i++)
    {
        fprintf(csvFile, ",%.2f", sensorValues[i]);
    }
    fprintf(csvFile, "\n");

    fflush(csvFile);
    return 0;
}

int main()
{
    // Open CSV file
    csvFile = fopen("Arduino_Data.csv", "w");
    if (!csvFile)
    {
        fprintf(stderr, "Error opening log file.\n");
        return 1;
    }

    // Write CSV header (always 8 columns)
    fprintf(csvFile, "Timestamp");
    for (int i = 1; i <= NUM_CHANNELS; i++)
    {
        fprintf(csvFile, ",Sensor%d", i);
    }
    fprintf(csvFile, "\n");

    // Setup serial connection
    if (setup_serial_connection() != 0)
    {
        fprintf(stderr, "Error setting up serial connection.\n");
        fclose(csvFile);
        return 1;
    }

    // Start data logging
    clock_t startTime = clock();
    while (1)
    {
        if (read_and_process_data(startTime) < 0)
        {
            fprintf(stderr, "Error reading serial data.\n");
            break;
        }
#ifdef _WIN32
        Sleep(100);
#else
        usleep(100000);
#endif
    }

    fclose(csvFile);
    return 0;
}

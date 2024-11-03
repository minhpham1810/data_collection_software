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

#include "LeapC.h"
#include "ExampleConnection.h"

LEAP_CLOCK_REBASER clockSynchronizer;

double x_vals[1000];
int sensorValue1_data[1000];
int sensorValue2_data[1000];
int sensorValue3_data[1000];
int sensorValue4_data[1000];
int data_count = 0;
time_t startTime;

#ifdef _WIN32
HANDLE serial_port;
#else
int serial_port;
#endif

// Function prototypes
int setup_serial_connection();
int read_and_process_data(FILE *ser);

int main(int argc, char** argv) {
    LEAP_CONNECTION* connHandle = OpenConnection();
    FILE *fp;
    fp = fopen("handData.csv", "w");

    FILE *ser;
    ser = fopen("Arduino_Data.csv", "w");
    if (!ser) {
        fprintf(stderr, "Error opening Arduino_Data.csv\n");
        return 1;
    }

    if (setup_serial_connection() != 0) {
        fprintf(stderr, "Error setting up serial connection\n");
        fclose(fp);
        fclose(ser);
        return 1;
    }

    while(!IsConnected){
        #ifdef _WIN32
        Sleep(250);
        #else
        usleep(250000);
        #endif
    }

    printf("Connected.\n");
    LeapCreateClockRebaser(&clockSynchronizer);
    clock_t cpuTime;
    int64_t targetFrameTime = 0;
    uint64_t targetFrameSize = 0;
    eLeapRS result;

    fprintf(fp,"Timestamp|Palm|ThumbStart|ThumbMid|ThumbEnd|IndexStart|IndexMid1|IndexMid2|IndexEnd|MiddleStart|MiddleMid1|MiddleMid2|MiddleEnd|RingStart|RingMid1|RingMid2|RingEnd|PinkyStart|PinkyMid1|PinkyMid2|PinkyEnd|Arm SecondHand --> \n");

    for (;;) {
        cpuTime = (clock_t).000001 * clock() / CLOCKS_PER_SEC;
        LeapUpdateRebase(clockSynchronizer, cpuTime, LeapGetNow());
        #ifdef _WIN32
        Sleep(10);
        #else
        usleep(10000);
        #endif
        cpuTime = (clock_t).000001 * clock() / CLOCKS_PER_SEC;
        LeapRebaseClock(clockSynchronizer, cpuTime, &targetFrameTime);
        result = LeapGetFrameSize(*connHandle, targetFrameTime, &targetFrameSize);
        
        if (result == eLeapRS_Success) {
            LEAP_TRACKING_EVENT* interpolatedFrame = malloc((size_t)targetFrameSize);
            result = LeapInterpolateFrame(*connHandle, targetFrameTime, interpolatedFrame, targetFrameSize);
            if (result == eLeapRS_Success) {
                exportHand(interpolatedFrame, fp);
                free(interpolatedFrame);
            } else {
                printf("LeapInterpolateFrame() result was %s.\n", ResultString(result));
            }
        } else {
            printf("LeapGetFrameSize() result was %s.\n", ResultString(result));
        }

        // read_and_process_data(ser);
    }

    fclose(fp);
    fclose(ser);
    return 0;
}

int exportHand(LEAP_TRACKING_EVENT* frame,FILE *fp) {

  // Timestamp
  // can also use LeapGetNow()
  fprintf(fp,"%lli",frame->info.timestamp);
  
  LEAP_HAND hand;
  LEAP_PALM palm;
  LEAP_DIGIT finger;
  LEAP_BONE arm;
  // Both hands
  for(uint32_t h = 0; h < frame->nHands; h++){
    
    hand = frame->pHands[h];

    // Palm
    palm = hand.palm;
    fprintf(fp,"|[%f,%f,%f]",hand.palm.position.x,palm.position.y,palm.position.z);

    // Fingers
    for (uint32_t i = 0; i < 5; i++) {
      finger = hand.digits[i];

      // iterate through every bone
      for (uint32_t j = 0; j < 4; j++) {
        fprintf(fp,"|[[%f,%f,%f],[%f,%f,%f]]",
                      finger.bones[j].prev_joint.x,finger.bones[j].next_joint.x,
                      finger.bones[j].prev_joint.y,finger.bones[j].next_joint.y,
                      finger.bones[j].prev_joint.z,finger.bones[j].next_joint.z
                );
      }
    }

    // Arm
    arm = hand.arm;
    fprintf(fp,"|[[%f,%f,%f],[%f,%f,%f]]",arm.prev_joint.x,arm.prev_joint.y,arm.prev_joint.z,
                                        arm.next_joint.x,arm.next_joint.y,arm.next_joint.z);
  }
  fprintf(fp,"\n");
  return 0;
}

int setup_serial_connection() {
    #ifdef _WIN32
    serial_port = CreateFile("\\\\.\\COM5", GENERIC_READ, 0, NULL, OPEN_EXISTING, 0, NULL);
    if (serial_port == INVALID_HANDLE_VALUE) {
        fprintf(stderr, "Error opening COM port\n");
        return -1;
    }
    DCB dcbSerialParams = {0};
    dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
    dcbSerialParams.BaudRate = CBR_9600;
    dcbSerialParams.ByteSize = 8;
    dcbSerialParams.StopBits = ONESTOPBIT;
    dcbSerialParams.Parity = NOPARITY;
    SetCommState(serial_port, &dcbSerialParams);
    #else
    serial_port = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY);
    if (serial_port < 0) {
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
    return 0;
}

int read_and_process_data(FILE *ser) {
    char line[256];
    #ifdef _WIN32
    DWORD bytes_read;
    ReadFile(serial_port, line, sizeof(line) - 1, &bytes_read, NULL);
    line[bytes_read] = '\0';
    #else
    int n = read(serial_port, line, sizeof(line) - 1);
    if (n < 0) return -1;
    line[n] = '\0';
    #endif

    int sensorValues[4];
    int i = 0;
    char *token = strtok(line, " ");
    while (token != NULL && i < 4) {
        sensorValues[i++] = atoi(token);
        token = strtok(NULL, " ");
    }

    time_t currentTime = time(NULL);
    x_vals[data_count] = difftime(currentTime, startTime);
    sensorValue1_data[data_count] = sensorValues[0];
    sensorValue2_data[data_count] = sensorValues[1];
    sensorValue3_data[data_count] = sensorValues[2];
    sensorValue4_data[data_count] = sensorValues[3];
    data_count++;

    fprintf(ser, "%.2f,%d,%d,%d,%d\n",
           x_vals[data_count - 1], sensorValues[0], sensorValues[1], sensorValues[2], sensorValues[3]);
    return 0;
}

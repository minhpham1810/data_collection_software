#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>


#ifdef _WIN32
#include <windows.h>
DWORD WINAPI handle_ultraleap_data(LPVOID arg);
DWORD WINAPI handle_arduino_data(LPVOID arg);
#else
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <pthread.h>
void* handle_ultraleap_data(void* arg);
void* handle_arduino_data(void* arg);
#endif

#include "LeapC.h"
#include "ExampleConnection.h"
#include "cJSON.h"

//Global variables for Ultraleap
LEAP_CLOCK_REBASER clockSynchronizer;

//Global variables for Muscle Sensors
double x_vals[1000];
int sensorValue1_data[1000], sensorValue2_data[1000], sensorValue3_data[1000], sensorValue4_data[1000];
int data_count = 0;

#ifdef _WIN32
HANDLE serial_port;
#else
int serial_port;
#endif

// File pointers for data storage
FILE *fp;
FILE *ser;
FILE *fpcsv;

// Main function - creates threads and opens files
int main(int argc, char** argv) {

    char c;
    printf("Press Enter to begin collecting data.\n"); 
            // Once data collection has begun, press Enter to stop collecting data.\n");
    while ((c = getchar()) != '\n');  // Wait until Enter key is pressed

    // Open files
    fpcsv = fopen("handData.csv", "w");
    fp = fopen("handData.json", "w");
    ser = fopen("Arduino_Data.csv", "w");
    if (!fp || !ser || !fpcsv) {
        fprintf(stderr, "Error opening data files.\n");
        return 1;
    }

    // Setup serial connection to Arduino
    if (setup_serial_connection() != 0) {
        fprintf(stderr, "Error setting up serial connection\n");
        fclose(fp);
        fclose(ser);
        return 1;
    }

    // Create Threads
#ifdef _WIN32
    HANDLE ultraleap_thread = CreateThread(NULL, 0, handle_ultraleap_data, NULL, 0, NULL);
    HANDLE arduino_thread = CreateThread(NULL, 0, handle_arduino_data, NULL, 0, NULL);

    WaitForSingleObject(ultraleap_thread, INFINITE);
    WaitForSingleObject(arduino_thread, INFINITE);

    CloseHandle(ultraleap_thread);
    CloseHandle(arduino_thread);
#else
    pthread_t ultraleap_thread, arduino_thread;
    pthread_create(&ultraleap_thread, NULL, handle_ultraleap_data, NULL);
    pthread_create(&arduino_thread, NULL, handle_arduino_data, NULL);
    pthread_join(ultraleap_thread, NULL);
    pthread_join(arduino_thread, NULL);
#endif

    fclose(fp);
    fclose(ser);
    return 0;
}

// Thread function for handling Ultraleap data
#ifdef _WIN32
DWORD WINAPI handle_ultraleap_data(LPVOID arg) {
#else
void* handle_ultraleap_data(void* arg) {
#endif

    LEAP_CONNECTION* connHandle = OpenConnection();
    
    LeapCreateClockRebaser(&clockSynchronizer);
    //start clock
    clock_t startTime = clock();

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

                //Print Hand Data if we have it
                if (interpolatedFrame->nHands > 0) {
                    exportHand(interpolatedFrame, fpcsv, startTime);
                    append_frame_to_json(interpolatedFrame,fp,startTime);
                }
                free(interpolatedFrame);
                
            } else {
                printf("LeapInterpolateFrame() result was %s.\n", ResultString(result));
            }
        } else {
            printf("LeapGetFrameSize() result was %s.\n", ResultString(result));
        }
        
        
    }

    fclose(fp);
    return 0;
}

// Function to get the current timestamp
long long get_timestamp(clock_t startTime) {
    clock_t nowTime = clock(); 
    return (double)(nowTime - startTime) / CLOCKS_PER_SEC;
}

// Function to convert a LEAP_VECTOR to cJSON
cJSON* vector_to_json(LEAP_VECTOR vec) {
    cJSON* json_vec = cJSON_CreateObject();
    cJSON_AddNumberToObject(json_vec, "x", vec.x);
    cJSON_AddNumberToObject(json_vec, "y", vec.y);
    cJSON_AddNumberToObject(json_vec, "z", vec.z);
    return json_vec;
}

// Function to convert a LEAP_QUATERNION to cJSON
cJSON* quaternion_to_json(LEAP_QUATERNION quat) {
    cJSON* json_quat = cJSON_CreateObject();
    cJSON_AddNumberToObject(json_quat, "x", quat.x);
    cJSON_AddNumberToObject(json_quat, "y", quat.y);
    cJSON_AddNumberToObject(json_quat, "z", quat.z);
    cJSON_AddNumberToObject(json_quat, "w", quat.w);
    return json_quat;
}

// Function to convert a LEAP_BONE to cJSON
cJSON* bone_to_json(LEAP_BONE bone) {
    cJSON* json_bone = cJSON_CreateObject();
    cJSON_AddItemToObject(json_bone, "prev_joint", vector_to_json(bone.prev_joint));
    cJSON_AddItemToObject(json_bone, "next_joint", vector_to_json(bone.next_joint));
    cJSON_AddNumberToObject(json_bone, "width", bone.width);
    cJSON_AddItemToObject(json_bone, "rotation", quaternion_to_json(bone.rotation));
    return json_bone;
}

// Function to convert a LEAP_HAND to cJSON
cJSON* hand_to_json(LEAP_HAND hand) {
    cJSON* json_hand = cJSON_CreateObject();
    cJSON_AddNumberToObject(json_hand, "id", hand.id);
    cJSON_AddNumberToObject(json_hand, "type", hand.type);
    cJSON_AddNumberToObject(json_hand, "confidence", hand.confidence);
    cJSON_AddNumberToObject(json_hand, "visible_time", hand.visible_time);
    cJSON_AddItemToObject(json_hand, "palm_position", vector_to_json(hand.palm.position));
    cJSON_AddItemToObject(json_hand, "palm_velocity", vector_to_json(hand.palm.velocity));
    cJSON_AddItemToObject(json_hand, "palm_normal", vector_to_json(hand.palm.normal));
    cJSON_AddItemToObject(json_hand, "palm_orientation", quaternion_to_json(hand.palm.orientation));
    cJSON_AddNumberToObject(json_hand, "grab_strength", hand.grab_strength);
    cJSON_AddNumberToObject(json_hand, "pinch_strength", hand.pinch_strength);
    cJSON_AddItemToObject(json_hand, "arm", bone_to_json(hand.arm));
    return json_hand;
}

// Function to append a frame to the JSON file
void append_frame_to_json(LEAP_TRACKING_EVENT* frame, FILE *fp, clock_t startTime) {
    if (frame->nHands == 0) return;

    cJSON* json_data;
    fseek(fp, 0, SEEK_SET);  // Ensure we're at the start of the file.
    
    // Read the current content of the file
    char buffer[1024];
    fread(buffer, sizeof(buffer), 1, fp);
    json_data = cJSON_Parse(buffer);
    
    // If no data or invalid, initialize an empty structure
    if (!json_data) {
        json_data = cJSON_CreateObject();
        cJSON_AddArrayToObject(json_data, "frames");
    }

    // Prepare the frames array and new frame object
    cJSON* json_frames = cJSON_GetObjectItem(json_data, "frames");
    cJSON* json_frame = cJSON_CreateObject();
    cJSON* json_hands = cJSON_CreateArray();

    // Calculate timestamp from start time
    long long timestamp = get_timestamp(startTime);

    // Set the timestamp for this frame
    cJSON_AddNumberToObject(json_frame, "timestamp", timestamp);

    // Loop over the hands in the frame and add them to the hands array
    for (int i = 0; i < frame->nHands; i++) {
        LEAP_HAND hand = frame->pHands[i];
        if (hand.id != 0) {
            cJSON_AddItemToArray(json_hands, hand_to_json(hand));
        }
    }

    // Add the hands data to the frame
    cJSON_AddItemToObject(json_frame, "hands", json_hands);

    // Add the frame to the frames array
    cJSON_AddItemToArray(json_frames, json_frame);

    // Go to the beginning of the file to overwrite it
    fseek(fp, 0, SEEK_SET);

    // Print the JSON structure into the file
    char* json_string = cJSON_PrintPreallocated(json_data, 1024 * 1024, 1);
    fprintf(fp, "%s", json_string);
    fclose(fp);

    // Clean up
    cJSON_Delete(json_data);
}

// Export hand data to CSV
int exportHand(LEAP_TRACKING_EVENT* frame,FILE *fp,clock_t startTime) {

  clock_t nowTime = clock();  
  // Timestamp
  // can also use LeapGetNow()
  fprintf(fp,"%f",(double)(nowTime - startTime) / CLOCKS_PER_SEC);
  
  LEAP_HAND hand;
  LEAP_PALM palm;
  LEAP_DIGIT finger;
  LEAP_BONE arm;
  // Both hands
  for(uint32_t h = 0; h < frame->nHands; h++){
    
    hand = frame->pHands[h];

    // Palm
    palm = hand.palm;
    fprintf(fp,"|[[%f,%f,%f],[%f,%f,%f,%f]]",hand.palm.position.x,palm.position.y,palm.position.z,
                             palm.orientation.x,palm.orientation.y,palm.orientation.z,palm.orientation.w);

    // Fingers
    for (uint32_t i = 0; i < 5; i++) {
      finger = hand.digits[i];

      // iterate through every bone
      for (uint32_t j = 0; j < 4; j++) {
        fprintf(fp,"|[[%f,%f,%f],[%f,%f,%f],[%f,%f,%f,%f]]",
                      finger.bones[j].prev_joint.x, finger.bones[j].prev_joint.y,finger.bones[j].prev_joint.z,
                      finger.bones[j].next_joint.x,finger.bones[j].next_joint.y,finger.bones[j].next_joint.z,
                      finger.bones[j].rotation.x,finger.bones[j].rotation.y,finger.bones[j].rotation.z,finger.bones[j].rotation.w        
                );
      }
    }

    // Arm
    arm = hand.arm;
    fprintf(fp,"|[[%f,%f,%f],[%f,%f,%f],[%f,%f,%f,%f]]",arm.prev_joint.x,arm.prev_joint.y,arm.prev_joint.z,
                                        arm.next_joint.x,arm.next_joint.y,arm.next_joint.z,
                                        arm.rotation.x,arm.rotation.y,arm.rotation.z,arm.rotation.w);
  }
  fprintf(fp,"\n");
  return 0;
}

// Thread function for handling Arduino data
#ifdef _WIN32
DWORD WINAPI handle_arduino_data(LPVOID arg) {
#else
void* handle_arduino_data(void* arg) {
#endif
    //start clock
    clock_t startTime = clock();

    for (;;) {
        read_and_process_data(ser,startTime);
// #ifdef _WIN32
//         Sleep(100);
// #else
//         usleep(100000);
// #endif
    }
#ifdef _WIN32
    return 0;
#else
    return NULL;
#endif
}

int setup_serial_connection() {
    #ifdef _WIN32
    serial_port = CreateFile("\\\\.\\COM3", GENERIC_READ, 0, NULL, OPEN_EXISTING, 0, NULL);
    if (serial_port == INVALID_HANDLE_VALUE) {
        fprintf(stderr, "Error opening COM port; check port number\n");
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

int read_and_process_data(FILE *ser,clock_t startTime) {

    clock_t nowTime = clock();  
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

    int activeSensors[9];


    int sensorValues[9];
    int i = 0;
    char *token = strtok(line, " ");
    while (token != NULL && i < 9) {
        sensorValues[i++] = atoi(token);
        token = strtok(NULL, " ");
    }
    x_vals[data_count] = (double)(nowTime - startTime) / CLOCKS_PER_SEC;
    sensorValue1_data[data_count] = sensorValues[0];
    sensorValue2_data[data_count] = sensorValues[1];
    sensorValue3_data[data_count] = sensorValues[2];
    sensorValue4_data[data_count] = sensorValues[3];
    sensorValue4_data[data_count] = sensorValues[4];
    sensorValue4_data[data_count] = sensorValues[5];
    sensorValue4_data[data_count] = sensorValues[6];
    sensorValue4_data[data_count] = sensorValues[7];
    sensorValue4_data[data_count] = sensorValues[8];
    
    data_count++;

    for (i = 0; i < activeSensors; i++)
    fprintf(ser, "%.2f,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
           x_vals[data_count - 1], sensorValues[0], sensorValues[1], sensorValues[2], sensorValues[3], sensorValues[4], sensorValues[5], sensorValues[6], sensorValues[7], sensorValues[8]);
    return 0;
}
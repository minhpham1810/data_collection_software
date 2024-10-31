/* Copyright (C) 2012-2017 Ultraleap Limited. All rights reserved.
 *
 * Use of this code is subject to the terms of the Ultraleap SDK agreement
 * available at https://central.leapmotion.com/agreements/SdkAgreement unless
 * Ultraleap has signed a separate license agreement with you or your
 * organisation.
 *
 */

#include <stdio.h>
#include <stdlib.h>

#ifdef _WIN32
#include <Windows.h>
#else
#include <unistd.h>
#endif

#include <time.h>
#include "LeapC.h"
#include "ExampleConnection.h"

LEAP_CLOCK_REBASER clockSynchronizer;

int main(int argc, char** argv) {
  LEAP_CONNECTION* connHandle = OpenConnection();

  FILE *fp;
  fp = fopen("handData.csv", "w");

  while(!IsConnected){
    millisleep(250);
  }

  printf("Connected.\n");
  //Create the clock synchronizer
  LeapCreateClockRebaser(&clockSynchronizer);
  clock_t cpuTime;
  int64_t targetFrameTime = 0;
  uint64_t targetFrameSize = 0;
  eLeapRS result;

  fprintf(fp,"Timestamp|Palm|ThumbStart|ThumbMid|ThumbEnd|IndexStart|IndexMid1|IndexMid2|IndexEnd|MiddleStart|MiddleMid1|MiddleMid2|MiddleEnd|RingStart|RingMid1|RingMid2|RingEnd|PinkyStart|PinkyMid1|PinkyMid2|PinkyEnd|Arm SecondHand --> \n");
  
  for(;;){
    //Calculate the application time
    cpuTime = (clock_t).000001 * clock()/CLOCKS_PER_SEC;//microseconds
    //Synchronize the clocks
    LeapUpdateRebase(clockSynchronizer, cpuTime, LeapGetNow());

    //Simulate delay (i.e. processing load, v-sync, etc)
    millisleep(10);

    //Now get the updated application time
    cpuTime = (clock_t) .000001 * clock()/CLOCKS_PER_SEC;

    //Translate application time to Leap time
    LeapRebaseClock(clockSynchronizer, cpuTime, &targetFrameTime);

    //Get the buffer size needed to hold the tracking data
    result = LeapGetFrameSize(*connHandle, targetFrameTime, &targetFrameSize);
    if(result == eLeapRS_Success){
      //Allocate enough memory
      LEAP_TRACKING_EVENT* interpolatedFrame = malloc((size_t)targetFrameSize);
      //Get the frame
      result = LeapInterpolateFrame(*connHandle, targetFrameTime, interpolatedFrame, targetFrameSize);
      if(result == eLeapRS_Success){
        //Use the data...
        exportHand(interpolatedFrame,fp);
        
        //Free the allocated buffer when done.
        free(interpolatedFrame);
      }
      else {
        printf("LeapInterpolateFrame() result was %s.\n", ResultString(result));
      }
    }
    else {
      printf("LeapGetFrameSize() result was %s.\n", ResultString(result));
    }
  } //ctrl-c to exit

  fclose(fp);
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
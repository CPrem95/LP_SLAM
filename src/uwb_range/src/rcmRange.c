//_____________________________________________________________________________
//
// Copyright 2011-2015 Time Domain Corporation
// Modified by 2024 Charith Premachandra
//
// This can communicate with the RCM over the USB interface (which acts like a serial port).
// Publishes over ROS2 the range values of the UWB nodes.
//_____________________________________________________________________________


//_____________________________________________________________________________
//
// #includes 
//_____________________________________________________________________________

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "rcmIf.h"
#include "rcm.h"

#include <stdio.h>
// #include <std_msgs/msg/string.h>
#include <std_msgs/msg/int16.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <unistd.h>

// these data structures for the publisher and subscriber are global, so that
// they can be configured in main() and can be used in the corresponding callback.
rcl_publisher_t my_pub;
std_msgs__msg__Int16 pub_msg;

int range1;
int range2;
int range3;

const int destNodeId1 = 102;
const int destNodeId2 = 103;
const int destNodeId3 = 105;
const int NodeIds[] = {destNodeId1, destNodeId2, destNodeId3};

int status;
// int freq = 20; // Publishing frequency
int time_period = 50; // 1000/freq

rcmIfType   rcmIf;
rcmConfiguration config;
rcmMsg_GetStatusInfoConfirm statusInfo;
rcmMsg_FullRangeInfo rangeInfo;
rcmMsg_DataInfo dataInfo;
rcmMsg_ScanInfo scanInfo;
rcmMsg_FullScanInfo fullScanInfo;

void my_ranging(int destNodeId, int * range_val, int * range_err, int * range_noise, int * range_vPeak, 
    rcmMsg_FullRangeInfo rangeInfo, rcmMsg_DataInfo dataInfo, rcmMsg_ScanInfo scanInfo, rcmMsg_FullScanInfo fullScanInfo)
{
if (rcmRangeTo(destNodeId, RCM_ANTENNAMODE_TXA_RXA, 0, NULL,
            &rangeInfo, &dataInfo, &scanInfo, &fullScanInfo) == 0)
    {
        if (0)
        {
        // we always get a range info packet
        printf("RANGE_INFO: responder %d, msg ID %u, range status %d, "
                "stopwatch %d ms, noise %d, vPeak %d, measurement type %d\n",
                rangeInfo.responderId, rangeInfo.msgId, rangeInfo.rangeStatus,
                rangeInfo.stopwatchTime, rangeInfo.noise, rangeInfo.vPeak,
                rangeInfo.rangeMeasurementType);

        // The RANGE_INFO can provide multiple types of ranges
        if (rangeInfo.rangeMeasurementType & RCM_RANGE_TYPE_PRECISION)
        {
            printf("Precision range: %d mm, error estimate %d mm\n",
                    rangeInfo.precisionRangeMm, rangeInfo.precisionRangeErrEst);
        }

        if (rangeInfo.rangeMeasurementType & RCM_RANGE_TYPE_COARSE)
        {
            printf("Coarse range: %d mm, error estimate %d mm\n",
                    rangeInfo.coarseRangeMm, rangeInfo.coarseRangeErrEst);
        }

        if (rangeInfo.rangeMeasurementType & RCM_RANGE_TYPE_FILTERED)
        {
            printf("Filtered range: %d mm, error estimate %d mm\n",
                    rangeInfo.filteredRangeMm, rangeInfo.filteredRangeErrEst);
            // printf("Filtered velocity: %d mm/s, error estimate %d mm/s\n",
            //         rangeInfo.filteredRangeVel, rangeInfo.filteredRangeVelErrEst);
        }
        printf("\n");
        }
        *range_val = rangeInfo.filteredRangeMm;
        *range_err = rangeInfo.filteredRangeErrEst;
        *range_noise = rangeInfo.noise;
        *range_vPeak = rangeInfo.vPeak;
    }
}


//Not working, need to edit the func definitions 
//void my_ranging_all(int destNodeId1, int destNodeId2, int destNodeId3, rcmMsg_FullRangeInfo rangeInfo, 
//     rcmMsg_DataInfo dataInfo, rcmMsg_ScanInfo scanInfo, rcmMsg_FullScanInfo fullScanInfo)
//     {   
//         my_ranging(destNodeId1, &range1, rangeInfo, dataInfo, scanInfo, fullScanInfo);
//         my_ranging(destNodeId2, &range2, rangeInfo, dataInfo, scanInfo, fullScanInfo);
//         my_ranging(destNodeId3, &range3, rangeInfo, dataInfo, scanInfo, fullScanInfo);
//     }

// int moving_avg

/***************************** CALLBACKS ***********************************/

void my_timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  rcl_ret_t rc;
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    rc = rcl_publish(&my_pub, &pub_msg, NULL);
    if (rc == RCL_RET_OK) {
      printf("Published message %d\n", pub_msg.data);
    } else {
      printf("timer_callback: Error publishing message %d\n", pub_msg.data);
    }
  } else {
    printf("timer_callback Error: timer parameter is NULL\n");
  }
  printf("Responder#1_INFO: range %d [mm]\n", range1);
  printf("Responder#2_INFO: range %d [mm]\n", range2);
  printf("Responder#3_INFO: range %d [mm]\n\n", range3);
}

//_____________________________________________________________________________
//
// local function prototypes
//_____________________________________________________________________________

void usage(void)
{
    printf("usage: rcmSampleApp -i <IP address> | -s <COM port> | -u <USB COM port>\n");
    printf("\nTo connect to radio at IP address 192.168.1.100 via Ethernet:\n");
    printf("\trcmSampleApp -i 192.168.1.100\n");

#ifdef WIN32
    printf("\nTo connect to radio's serial port at COM3:\n");
    printf("\trcmSampleApp -s COM3\n");
    printf("\nTo connect to radio's USB port at COM10:\n");
    printf("\trcmSampleApp -u COM10\n");
#else
    printf("\nTo connect to radio's serial port at /dev/ttyUSB0:\n");
    printf("\trcmSampleApp -s /dev/ttyUSB0\n");
    printf("\nTo connect to radio's USB port at /dev/ttyACM0:\n");
    printf("\trcmSampleApp -u /dev/ttyACM0\n");
#endif

	exit(0);
}


//_____________________________________________________________________________
//
// main - sample app entry point
//_____________________________________________________________________________

int main(int argc, char *argv[], const char *argp[])
{
	printf("RCM ranging App\n\n");

    // check command line arguments

    if (!strcmp(argv[1], "-i"))
        rcmIf = rcmIfIp;
    else if (!strcmp(argv[1], "-s"))
        rcmIf = rcmIfSerial;
    else if (!strcmp(argv[1], "-u"))
        rcmIf = rcmIfUsb;
    else
        usage();

    // initialize the interface to the RCM
    if (rcmIfInit(rcmIf, argv[2]) != OK)
    {
        printf("Initialization failed.\n");
        exit(0);
    }

    // Make sure RCM is awake
    if (rcmSleepModeSet(RCM_SLEEP_MODE_ACTIVE) != 0)
    {
        printf("Time out waiting for sleep mode set.\n");
        exit(0);
    }

    // Make sure opmode is RCM
    if (rcmOpModeSet(RCM_OPMODE_RCM) != 0)
    {
        printf("Time out waiting for opmode set.\n");
        exit(0);
    }

    // execute Built-In Test - verify that radio is healthy
    if (rcmBit(&status) != 0)
    {
        printf("Time out waiting for BIT.\n");
        exit(0);
    }

    if (status != OK)
    {
        printf("Built-in test failed - status %d.\n", status);
        exit(0);
    }
    else
    {
        printf("Radio passes built-in test.\n\n");
    }

    // retrieve config from RCM
    if (rcmConfigGet(&config) != 0)
    {
        printf("Time out waiting for config confirm.\n");
        exit(0);
    }

    // print out configuration
    printf("Configuration:\n");
    printf("\tnodeId: %d\n", config.nodeId);
    printf("\tintegrationIndex: %d\n", config.integrationIndex);
    printf("\tantennaMode: %d\n", config.antennaMode);
    printf("\tcodeChannel: %d\n", config.codeChannel);
    printf("\telectricalDelayPsA: %d\n", config.electricalDelayPsA);
    printf("\telectricalDelayPsB: %d\n", config.electricalDelayPsB);
    printf("\tflags: 0x%X\n", config.flags);
    printf("\ttxGain: %d\n", config.txGain);

    // retrieve status/info from RCM
    if (rcmStatusInfoGet(&statusInfo) != 0)
    {
        printf("Time out waiting for status info confirm.\n");
        exit(0);
    }

    // print out status/info
    printf("\nStatus/Info:\n");
    printf("\tPackage version: %s\n", statusInfo.packageVersionStr);
    printf("\tRCM version: %d.%d build %d\n", statusInfo.appVersionMajor,
            statusInfo.appVersionMinor, statusInfo.appVersionBuild);
    printf("\tUWB Kernel version: %d.%d build %d\n", statusInfo.uwbKernelVersionMajor,
            statusInfo.uwbKernelVersionMinor, statusInfo.uwbKernelVersionBuild);
    printf("\tFirmware version: %x/%x/%x ver %X\n", statusInfo.firmwareMonth,
            statusInfo.firmwareDay, statusInfo.firmwareYear,
            statusInfo.firmwareVersion);
    printf("\tSerial number: %08X\n", statusInfo.serialNum);
    printf("\tBoard revision: %c\n", statusInfo.boardRev);
    printf("\tTemperature: %.2f degC\n\n", statusInfo.temperature/4.0);

    //______________________________________________________________________________________________
    // Publisher CODE 
    //______________________________________________________________________________________________
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    rcl_ret_t rc;

    const unsigned int n_topics = 3;
    const char * topic_names_value[] = {"value1", "value2", "value3"};
    const char * topic_names_error[] = {"error1", "error2", "error3"};
    const char * topic_names_noise[] = {"noise1", "noise2", "noise3"};
    const char * topic_names_vPeak[] = {"vPeak1", "vPeak2", "vPeak3"};
    int range_val[] = {0, 0, 0};
    int range_err[] = {0, 0, 0};
    int range_noise[] = {0, 0, 0};
    int range_vPeak[] = {0, 0, 0};

    rcl_publisher_t publisher_range_val[n_topics];
    rcl_publisher_t publisher_range_err[n_topics];
    rcl_publisher_t publisher_range_noise[n_topics];
    rcl_publisher_t publisher_range_vPeak[n_topics];
    
    std_msgs__msg__Int16 pub_msgs[n_topics];


    // create init_options
    rc = rclc_support_init(&support, argc, argp, &allocator);
    if (rc != RCL_RET_OK) {
        printf("Error rclc_support_init.\n");
        return -1;
    }

    // create rcl_node
    rcl_node_t my_node = rcl_get_zero_initialized_node();
    rc = rclc_node_init_default(&my_node, "UWB_range_node", "ranges", &support);
    if (rc != RCL_RET_OK) {
        printf("Error in rclc_node_init_default\n");
        return -1;
    }

    const rosidl_message_type_support_t * my_type_support =
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16);

    ////////////////////////////////////////////////////////////////////////////
    //initialise each publisher 
    ////////////////////////////////////////////////////////////////////////////
    
    // Range value publishers
    for (unsigned int i = 0; i < n_topics; i++) {
        rc = rclc_publisher_init_default(
        &(publisher_range_val[i]),
        &my_node,
        my_type_support,
        topic_names_value[i]);
        if (RCL_RET_OK != rc) {
            printf("Error in rclc_publisher_init_default %s.\n", topic_names_value[i]);
            return -1;
        }
        // assign message to publisher
        std_msgs__msg__Int16__init(&pub_msgs[i]);
        pub_msgs[i].data= range_val[i];
    }
    // Range error publishers
    for (unsigned int i = 0; i < n_topics; i++) {
        rc = rclc_publisher_init_default(
        &(publisher_range_err[i]),
        &my_node,
        my_type_support,
        topic_names_error[i]);
        if (RCL_RET_OK != rc) {
            printf("Error in rclc_publisher_init_default %s.\n", topic_names_error[i]);
            return -1;
        }
        // assign message to publisher
        std_msgs__msg__Int16__init(&pub_msgs[i]);
        pub_msgs[i].data= range_err[i];
    }
    // Range noise publishers
    for (unsigned int i = 0; i < n_topics; i++) {
        rc = rclc_publisher_init_default(
        &(publisher_range_noise[i]),
        &my_node,
        my_type_support,
        topic_names_noise[i]);
        if (RCL_RET_OK != rc) {
            printf("Error in rclc_publisher_init_default %s.\n", topic_names_noise[i]);
            return -1;
        }
        // assign message to publisher
        std_msgs__msg__Int16__init(&pub_msgs[i]);
        pub_msgs[i].data= range_noise[i];
    }
    // Range vPeak publishers
    for (unsigned int i = 0; i < n_topics; i++) {
        rc = rclc_publisher_init_default(
        &(publisher_range_vPeak[i]),
        &my_node,
        my_type_support,
        topic_names_vPeak[i]);
        if (RCL_RET_OK != rc) {
            printf("Error in rclc_publisher_init_default %s.\n", topic_names_vPeak[i]);
            return -1;
        }
        // assign message to publisher
        std_msgs__msg__Int16__init(&pub_msgs[i]);
        pub_msgs[i].data= range_vPeak[i];
    }

    printf("Debug: publisher initialised\n");
    
    ////////////////////////////////////////////////////////////////////////////
    // Configuration of RCL Executor
    ////////////////////////////////////////////////////////////////////////////
    rclc_executor_t executor;
    executor = rclc_executor_get_zero_initialized_executor();
    // total number of handles = #subscriptions + #timers
    // Note:
    // If you need more than the default number of publisher/subscribers, etc., you
    // need to configure the micro-ROS middleware also!
    // See documentation in the executor.h at the function rclc_executor_init()
    // for more details.
    unsigned int num_handles = n_topics + 0;
    printf("Debug: number of DDS handles: %u\n", num_handles);
    rclc_executor_init(&executor, &support.context, num_handles, &allocator);

    // timeout specified in nanoseconds (here 1s)
    // rc = rclc_executor_spin_some(&executor, 1000 * (1000 * 1000));

    // for (unsigned int tick = 0; tick < 10; tick++) {
    while(1) {
        for (unsigned int i = 0; i < n_topics; i++) {
            my_ranging(NodeIds[i], &range_val[i], &range_err[i], &range_noise[i], &range_vPeak[i], rangeInfo, dataInfo, scanInfo, fullScanInfo);
            
            pub_msgs[i].data = range_val[i];
            rc = rcl_publish(&publisher_range_val[i], &pub_msgs[i], NULL);
            // Uncomment below to print values on screen
            // if (rc == RCL_RET_OK) {
            //     printf("\nRange from %s %d", topic_names_value[i], pub_msgs[i].data);
            // } else {
            //     printf("Error publishing message %d\n", pub_msgs[i].data);
            // }

            pub_msgs[i].data = range_err[i];
            rc = rcl_publish(&publisher_range_err[i], &pub_msgs[i], NULL);

            pub_msgs[i].data = range_noise[i];
            rc = rcl_publish(&publisher_range_noise[i], &pub_msgs[i], NULL);

            pub_msgs[i].data = range_vPeak[i];
            rc = rcl_publish(&publisher_range_vPeak[i], &pub_msgs[i], NULL);
            
        // // capture the message in the callback || timeout specified in nanoseconds (here 50 ms)
        // rc = rclc_executor_spin_some(&executor, 50 * (1000 * 1000));
        }
        printf("\n");      
    }

    // clean up
    rc = rclc_executor_fini(&executor);

    for (unsigned int i = 0; i < n_topics; i++) {
        rc += rcl_publisher_fini(&(publisher_range_val[i]), &my_node);
        rc += rcl_publisher_fini(&(publisher_range_err[i]), &my_node);
        rc += rcl_publisher_fini(&(publisher_range_noise[i]), &my_node);
        rc += rcl_publisher_fini(&(publisher_range_vPeak[i]), &my_node);
    }
    rc += rcl_node_fini(&my_node);
    rc += rclc_support_fini(&support);

    for (unsigned int i = 0; i < n_topics; i++) {
        std_msgs__msg__Int16__fini(&(pub_msgs[i]));
    }

    if (rc != RCL_RET_OK) {
        printf("Error while cleaning up!\n");
        return -1;
    }
    // perform cleanup RANGER
    rcmIfClose();
    return 0;
}
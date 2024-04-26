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
#include <rosidl_runtime_c/primitives_sequence.h>

#include <stdio.h>
// #include <std_msgs/msg/string.h>
#include <std_msgs/msg/int16_multi_array.h>
#include <rclc/executor.h>
#include <stdio.h>
#include <rclc/rclc.h>
#include <unistd.h>

// these data structures for the publisher and subscriber are global, so that
// they can be configured in main() and can be used in the corresponding callback.
rcl_publisher_t my_pub;
std_msgs__msg__Int16MultiArray pub_msg;

int16_t range1;
int16_t range2;
int16_t range3;

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

void my_ranging(int destNodeId, int16_t * range_val, rcmMsg_FullRangeInfo rangeInfo, 
    rcmMsg_DataInfo dataInfo, rcmMsg_ScanInfo scanInfo, rcmMsg_FullScanInfo fullScanInfo)
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
    }
}

void my_ranging_all(int destNodeId1, int destNodeId2, int destNodeId3, rcmMsg_FullRangeInfo rangeInfo, 
    rcmMsg_DataInfo dataInfo, rcmMsg_ScanInfo scanInfo, rcmMsg_FullScanInfo fullScanInfo)
    {
        my_ranging(destNodeId1, &range1, rangeInfo, dataInfo, scanInfo, fullScanInfo);
        my_ranging(destNodeId2, &range2, rangeInfo, dataInfo, scanInfo, fullScanInfo);
        my_ranging(destNodeId3, &range3, rangeInfo, dataInfo, scanInfo, fullScanInfo);
    }


/***************************** CALLBACKS ***********************************/

void my_timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  rcl_ret_t rc;
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    //printf("Timer: time since last call %d\n", (int) last_call_time);
    my_ranging_all(destNodeId1, destNodeId2, destNodeId3, rangeInfo, dataInfo, scanInfo, fullScanInfo);
    rc = rcl_publish(&my_pub, &pub_msg, NULL);
    if (rc == RCL_RET_OK) {
      printf("Published message %d\n", pub_msg.data.data[0]);
    } else {
      printf("timer_callback: Error publishing message %d\n", pub_msg.data.data[0]);
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
    // int destNodeId=DEFAULT_DEST_NODE_ID;
    my_ranging_all(destNodeId1, destNodeId2, destNodeId3, rangeInfo, dataInfo, scanInfo, fullScanInfo);

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
    const char* topic_names[] = {"anchor1", "anchor2", "anchor3"};
    int16_t range_vals[] = {0, 0, 0};
    printf("size of range_vals: %ld\n", sizeof(range_vals));

    rcl_publisher_t my_pubs[n_topics];
    // std_msgs__msg__Int16MultiArray pub_msgs[n_topics];
    std_msgs__msg__Int16MultiArray__Sequence pub_msgs[n_topics];

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
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16MultiArray);

    printf("Debug: node initialised\n");

    //initialise each publisher 
    for (unsigned int i = 0; i < n_topics; i++) {
        printf("Debug: publisher %s\n", topic_names[i]);
        rc = rclc_publisher_init_default(
        &(my_pubs[i]),
        &my_node,
        my_type_support,
        topic_names[i]);
        if (RCL_RET_OK != rc) {
            printf("Error in rclc_publisher_init_default %s.\n", topic_names[i]);
            return -1;
        }
        // assign message to publisher


        std_msgs__msg__Int16MultiArray__Sequence__init(&pub_msgs[i], 2);

        // std_msgs__msg__Int16MultiArray__init(&pub_msgs[i]);
        // std_msgs__msg__Int16MultiArray__Sequence__create(&pub_msgs[i], 8);
        
        // const unsigned int PUB_MSG_CAPACITY = 40;
        // const char* PUB_LABEL = "range";
        //pub_msgs[i].layout.dim.push_back(std_msgs__msg__MultiArrayDimension, allocator));
        // const unsigned int PUB_MSG_CAPACITY = 3;
        // const unsigned int PUB_MSG_STRIDE = 1;
        // rosidl_runtime_c__String LABEL;
        // LABEL.data = "range";

        printf("Debug: publisher %s initialised\n", topic_names[i]);
        // allocator.allocate(&pub_msgs[i].layout.dim.data->size, PUB_MSG_CAPACITY, allocator.state);
        // allocator.reallocate(&pub_msgs[i].layout.dim.data->stride, PUB_MSG_STRIDE, allocator.state);

        pub_msgs[i].data->data.data = (int16_t *)malloc(sizeof(int16_t)*3);
        // pub_msgs[i].layout.data_offset = 0;

        // pub_msgs[i].layout.dim.data->size = PUB_MSG_CAPACITY;
        // pub_msgs[i].layout.dim->size = PUB_MSG_CAPACITY;
        // pub_msgs[i].layout.dim.data->stride = PUB_MSG_STRIDE;
        // pub_msgs[i].layout.dim.data->label = LABEL;
        // range_vals[i] = 0;

        printf("Debug: publisher %s initialised\n", topic_names[i]);
        // // pub_msgs[i].layout.dim.data->label = 
        // // pub_msgs[i].layout.dim.data->label = PUB_LABEL;

        // pub_msgs[i].data.capacity = PUB_MSG_CAPACITY;
        // pub_msgs[i].data.data[0]  = range_vals[i];
    }

    // printf("Debug: publisher initialised\n");

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
    // printf("Debug: number of DDS handles: %u\n", num_handles);
    rclc_executor_init(&executor, &support.context, num_handles, &allocator);

    // timeout specified in nanoseconds (here 1s)
    // rc = rclc_executor_spin_some(&executor, 1000 * (1000 * 1000));

    // for (unsigned int tick = 0; tick < 10; tick++) {

    while(1) {
        for (unsigned int i = 0; i < n_topics; i++) {
            my_ranging(NodeIds[i], &range_vals[i], rangeInfo, dataInfo, scanInfo, fullScanInfo);

            pub_msgs[i].data->data.data[0] = range_vals[i];
            printf("size of range_vals: %ld\n", sizeof(range_vals[i]));
            printf("size of pub_msgs[i].data->data.data: %ld\n", sizeof(pub_msgs[i].data->data.data[0]));
            
            //publish once for each topic
            rc = rcl_publish(&my_pubs[i], &pub_msgs[i], NULL);
            if (rc == RCL_RET_OK) {
                printf("\nRange from %s %d", topic_names[i], pub_msgs[i].data->data.data[0]);
            } else {
                printf("Error publishing message %d\n", pub_msgs[i].data->data.data[0]);
            }
        // // capture the message in the callback || timeout specified in nanoseconds (here 50 ms)
            // rc = rclc_executor_spin_some(&executor, 50 * (1000 * 1000));
        }
        printf("\n");      
    }

    // clean up
    rc = rclc_executor_fini(&executor);

    for (unsigned int i = 0; i < n_topics; i++) {
        rc += rcl_publisher_fini(&(my_pubs[i]), &my_node);
    }
    rc += rcl_node_fini(&my_node);
    rc += rclc_support_fini(&support);

    for (unsigned int i = 0; i < n_topics; i++) {
        std_msgs__msg__Int16MultiArray__Sequence__fini(&(pub_msgs[i]));
    }

    if (rc != RCL_RET_OK) {
        printf("Error while cleaning up!\n");
        return -1;
    }
    // perform cleanup RANGER
    rcmIfClose();
    return 0;
}
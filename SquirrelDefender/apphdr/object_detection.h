#pragma once

#ifdef ENABLE_CV

/********************************************************************************
 * @file    object_detection.h
 * @author  Cameron Rose
 * @date    6/7/2023
 ********************************************************************************/
#ifndef OBJECT_DETECTION_H
#define OBJECT_DETECTION_H

/********************************************************************************
 * Includes
 ********************************************************************************/
#include "common_inc.h"
#include "video_IO.h"
#include "parameters.h"

#ifdef JETSON_B01

#include "jetson-utils/videoSource.h"
#include "jetson-utils/videoOutput.h"
#include "jetson-inference/detectNet.h"
#include "jetson-inference/objectTracker.h"
#include <jetson-inference/objectTrackerIOU.h>

#elif _WIN32

#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <vector>
#include <string>
#include <fstream>

#else



#endif // JETSON_B01



/********************************************************************************
 * Imported objects
 ********************************************************************************/
#ifdef JETSON_B01

extern videoSource *input;
extern uchar3 *image;
extern uint32_t input_video_width;
extern uint32_t input_video_height;

#elif _WIN32

extern cv::Mat image;
extern float input_video_width;
extern float input_video_height;

#else



#endif // JETSON_B01

/********************************************************************************
 * Exported objects
 ********************************************************************************/
#ifdef JETSON_B01

extern detectNet *net;
extern detectNet::Detection *detections;

#elif _WIN32

extern cv::dnn::Net net;
extern std::vector<cv::Mat> detections;

#else



#endif // JETSON_B01

extern int numDetections;

/********************************************************************************
 * Function prototypes and Class Definitions
 ********************************************************************************/
class Detection
{
public:
    Detection(void);
    ~Detection(void);

    static void detection_loop(void);
    static bool detection_net_init(void);
    static void shutdown(void);
    static bool create_detection_network(void);
    static void detect_objects(void);
    static void get_object_info(void);
    static void print_object_info(void);
    static int print_usage(void);
    static void print_performance_stats(void);
    static void delete_tracking_net(void);

private:
};

class Target
{
public:
    Target();
    ~Target();

    int num_targets;
    int target_id;

private:
};

#endif // OBJECT_DETECTION_H

#endif // ENABLE_CV
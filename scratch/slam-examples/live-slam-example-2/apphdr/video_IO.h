#pragma once

/********************************************************************************
 * @file    video_IO.h
 * @author  Cameron Rose
 * @date    6/7/2023
 ********************************************************************************/
#ifndef VIDEO_IO_H
#define VIDEO_IO_H

/********************************************************************************
 * Includes
 ********************************************************************************/
#include "jetson-utils/videoSource.h"
#include "jetson-utils/videoOutput.h"
#include "jetson-inference/detectNet.h"
#include "jetson-inference/objectTracker.h"
#include <jetson-inference/objectTrackerIOU.h>
#include <string>
#include <fstream>

/********************************************************************************
 * Imported objects
 ********************************************************************************/
extern detectNet *net;

/********************************************************************************
 * Exported objects
 ********************************************************************************/
extern bool valid_image_rcvd;
extern videoSource *input;
extern uchar3 *image;
extern uint32_t input_video_width;
extern uint32_t input_video_height;

/********************************************************************************
 * Function prototypes and Class Definitions
 ********************************************************************************/
class Video
{
public:
    Video();
    ~Video();

    static bool video_output_file_reset(void);
    static bool video_init(void);
    static bool create_input_video_stream(void);
    static bool create_output_vid_stream(void);
    static bool create_display_video_stream(void);
    static void video_proc_loop(void);
    static void video_output_loop(void);
    static void shutdown(void);
    static bool capture_image(void);
    static bool save_video(void);
    static bool display_video(void);
    static void calc_video_res(void);
    static void delete_input_video_stream(void);
    static void delete_video_file_stream(void);
    static void delete_video_display_stream(void);

private:
};

#endif // VIDEO_IO_H

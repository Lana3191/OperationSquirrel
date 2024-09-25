#ifdef ENABLE_CV

/********************************************************************************
 * @file    target_tracking.cpp
 * @author  Cameron Rose
 * @date    6/7/2023
 * @brief   All methods needed to initialize and create a detection network and
            choose a target.
 ********************************************************************************/

/********************************************************************************
 * Includes
 ********************************************************************************/
#include "object_detection.h"

/********************************************************************************
 * Typedefs
 ********************************************************************************/

/********************************************************************************
 * Private macros and defines
 ********************************************************************************/

/********************************************************************************
 * Object definitions
 ********************************************************************************/
#ifdef JETSON_B01

detectNet *net;
detectNet::Detection *detections;

#elif _WIN32

cv::dnn::Net net;
std::vector<std::string> class_list;
std::vector<cv::Mat> detections;

#else



#endif // JETSON_B01

int numDetections;

/********************************************************************************
 * Calibration definitions
 ********************************************************************************/
#ifdef JETSON_B01

// Nothing to add

#elif _WIN32

 // Constants.
const float INPUT_WIDTH = 640.0;
const float INPUT_HEIGHT = 640.0;
const float SCORE_THRESHOLD = 0.5;
const float NMS_THRESHOLD = 0.45;
const float CONFIDENCE_THRESHOLD = 0.45;

// Text parameters.
const float FONT_SCALE = 0.7;
const int FONT_FACE = cv::FONT_HERSHEY_SIMPLEX;
const int THICKNESS = 1;

// Colors.
cv::Scalar BLACK = cv::Scalar(0, 0, 0);
cv::Scalar BLUE = cv::Scalar(255, 178, 50);
cv::Scalar YELLOW = cv::Scalar(0, 255, 255);
cv::Scalar RED = cv::Scalar(0, 0, 255);

#else



#endif // JETSON_B01

/********************************************************************************
 * Function definitions
 ********************************************************************************/

#ifdef _WIN32

void draw_label(cv::Mat& input, std::string label, int left, int top)
{
    // Display the label at the top of the bounding box.
    int baseLine;
    cv::Size label_size = cv::getTextSize(label, FONT_FACE, FONT_SCALE, THICKNESS, &baseLine);
    top = std::max(top, label_size.height);
    // Top left corner.
    cv::Point tlc = cv::Point(left, top);
    // Bottom right corner.
    cv::Point brc = cv::Point(left + label_size.width, top + label_size.height + baseLine);
    // Draw white rectangle.
    cv::rectangle(input, tlc, brc, BLACK, cv::FILLED);
    // Put the label on the black rectangle.
    cv::putText(input, label, cv::Point(left, top + label_size.height), FONT_FACE, FONT_SCALE, YELLOW, THICKNESS);
}

std::vector<cv::Mat> pre_process(cv::Mat& input, cv::dnn::Net& net)
{
    // Convert to blob.
    cv::Mat blob;
    cv::dnn::blobFromImage(input, blob, 1. / 255., cv::Size(INPUT_WIDTH, INPUT_HEIGHT), cv::Scalar(), true, false);

    net.setInput(blob);

    // Forward propagate.
    std::vector<cv::Mat> outputs;
    net.forward(outputs, net.getUnconnectedOutLayersNames());

    return outputs;
}

cv::Mat post_process(cv::Mat& input, std::vector<cv::Mat>& outputs, const std::vector<std::string>& class_name)
{
    // Initialize vectors to hold respective outputs while unwrapping detections.
    std::vector<int> class_ids;
    std::vector<float> confidences;
    std::vector<cv::Rect> boxes;
    // Resizing factor.
    float x_factor = input.cols / INPUT_WIDTH;
    float y_factor = input.rows / INPUT_HEIGHT;
    float* data = (float*)outputs[0].data;
    const int dimensions = 85;
    // 25200 for default size 640.
    const int rows = 25200;
    // Iterate through 25200 detections.
    for (int i = 0; i < rows; ++i)
    {
        float confidence = data[4];
        // Discard bad detections and continue.
        if (confidence >= CONFIDENCE_THRESHOLD)
        {
            float* classes_scores = data + 5;
            // Create a 1x85 Mat and store class scores of 80 classes.
            cv::Mat scores(1, class_name.size(), CV_32FC1, classes_scores);
            // Perform minMaxLoc and acquire the index of best class score.
            cv::Point class_id;
            double max_class_score;
            cv::minMaxLoc(scores, 0, &max_class_score, 0, &class_id);
            // Continue if the class score is above the threshold.
            if (max_class_score > SCORE_THRESHOLD)
            {
                // Store class ID and confidence in the pre-defined respective vectors.
                confidences.push_back(confidence);
                class_ids.push_back(class_id.x);
                // Center.
                float cx = data[0];
                float cy = data[1];
                // Box dimension.
                float w = data[2];
                float h = data[3];
                // Bounding box coordinates.
                int left = int((cx - 0.5 * w) * x_factor);
                int top = int((cy - 0.5 * h) * y_factor);
                int width = int(w * x_factor);
                int height = int(h * y_factor);
                // Store good detections in the boxes vector.
                boxes.push_back(cv::Rect(left, top, width, height));
            }
        }
        // Jump to the next row.
        data += 85;
    }

    // Perform Non-Maximum Suppression and draw predictions.
    std::vector<int> indices;
    cv::dnn::NMSBoxes(boxes, confidences, SCORE_THRESHOLD, NMS_THRESHOLD, indices);
    for (int i = 0; i < indices.size(); i++)
    {
        int idx = indices[i];
        cv::Rect box = boxes[idx];
        int left = box.x;
        int top = box.y;
        int width = box.width;
        int height = box.height;
        // Draw bounding box.
        cv::rectangle(input, cv::Point(left, top), cv::Point(left + width, top + height), BLUE, 3 * THICKNESS);
        // Get the label for the class name and its confidence.
        std::string label = cv::format("%.2f", confidences[idx]);
        label = class_name[class_ids[idx]] + ":" + label;
        // Draw class labels.
        draw_label(input, label, left, top);
    }
    return input;
}

#endif // _WIN32

/********************************************************************************
 * Function: Detection
 * Description: Class constructor
 ********************************************************************************/
Detection::Detection(void) {};

/********************************************************************************
 * Function: ~Detection
 * Description: Class destructor
 ********************************************************************************/
Detection::~Detection(void) {};

/********************************************************************************
 * Function: create_detection_network
 * Description: Initialize the network used for object detection.
 ********************************************************************************/
bool Detection::create_detection_network(void)
{
#ifdef JETSON_B01

#ifdef DEBUG_BUILD

    Parameters detection_params("../params.json");

    float detection_thresh = detection_params.get_float_param("Detection_tracking", "Detect_Thresh");
    uint32_t max_batch_size = detection_params.get_uint32_param("Detection_tracking", "Max_Batch_Size");
    uint32_t min_frames = detection_params.get_uint32_param("Detection_tracking", "Min_Frames");
    uint32_t drop_frames = detection_params.get_uint32_param("Detection_tracking", "Drop_Frames");
    float overlap_thresh = detection_params.get_float_param("Detection_tracking", "Overlap_Threshold");

#else

    float detection_thresh = (float)0.7;
    uint32_t max_batch_size = (uint32_t)4;
    uint32_t min_frames = (uint32_t)25;
    uint32_t drop_frames = (uint32_t)50;
    float overlap_thresh = (float)0.5;

#endif // DEBUG_BUILD

    const char *model = "../networks/SSD-Mobilenet-v2/ssd_mobilenet_v2_coco.uff";
    const char *class_labels = "../networks/SSD-Mobilenet-v2/ssd_coco_labels.txt";
    float thresh = (float)0.5;
    const char *input_blob = "Input";
    const char *output_blob = "NMS";
    Dims3 inputDims(3, 720, 1280);
    const char *output_count = "NMS_1";

    // net = detectNet::Create("SSD_Inception_V2", detection_thresh, max_batch_size);
    // net = detectNet::Create("SSD_Mobilenet_V2", detection_thresh, max_batch_size);
    net = detectNet::Create(model, class_labels, thresh, input_blob, inputDims, output_blob, output_count);
    net->SetTracker(objectTrackerIOU::Create(min_frames, drop_frames, overlap_thresh));

    if (!net)
    {
        LogError("detectnet:  failed to load detectNet model\n");
        return false;
    }

#elif _WIN32

    // Load class list.
    std::ifstream ifs("../../networks/yolov5m/coco.names");
    std::string line;

    while (std::getline(ifs, line))
    {
        class_list.push_back(line);
    }

    // Load model and enable GPU processing
    net = cv::dnn::readNet("../../networks/yolov5m/yolov5m.onnx");
    net.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
    net.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);

#else



#endif // JETSON_B01

    return true;
}

/********************************************************************************
 * Function: detect_objects
 * Description: Initialize the network used for object detection.
 ********************************************************************************/
void Detection::detect_objects(void)
{
#ifdef JETSON_B01

    uint32_t overlay_flags = 0;

    overlay_flags = overlay_flags | detectNet::OVERLAY_LABEL | detectNet::OVERLAY_CONFIDENCE | detectNet::OVERLAY_TRACKING | detectNet::OVERLAY_LINES;

    if (overlay_flags > 0 && image != NULL)
    {
        numDetections = net->Detect(image, input->GetWidth(), input->GetHeight(), &detections, overlay_flags);
    }
    else if (image != NULL)
    {
        numDetections = net->Detect(image, input->GetWidth(), input->GetHeight(), &detections);
    }
    else
    {
        // No other options
    }

#elif _WIN32



#else



#endif // JETSON_B01
}

/********************************************************************************
 * Function: get_object_info
 * Description: Obtain info about detected objects.
 ********************************************************************************/
void Detection::get_object_info(void)
{
#ifdef JETSON_B01

    if (numDetections > 0)
    {
        // LogVerbose("%i objects detected\n", numDetections);

        for (int n = 0; n < numDetections; n++)
        {
            float boxWidth = detections[n].Width();
            float boxHeight = detections[n].Height();
        }
    }

#elif _WIN32



#else



#endif // JETSON_B01
}

/********************************************************************************
 * Function: print_object_info
 * Description: Print info about detected objects.
 ********************************************************************************/
void Detection::print_object_info(void)
{
#ifdef JETSON_B01

    if (numDetections > 0)
    {
        LogVerbose("%i objects detected\n", numDetections);

        for (int n = 0; n < numDetections; n++)
        {
            if (detections[n].TrackID >= 0) // is this a tracked object?
            {
                if (detections[n].ClassID == 1 && detections[n].Confidence > 0.5)
                {
                    LogVerbose("\ndetected obj %i  class #%u (%s)  confidence=%f\n", n, detections[n].ClassID, net->GetClassDesc(detections[n].ClassID), detections[n].Confidence);
                    LogVerbose("bounding box %i  (%.2f, %.2f)  (%.2f, %.2f)  w=%.2f  h=%.2f\n", n, detections[n].Left, detections[n].Top, detections[n].Right, detections[n].Bottom, detections[n].Width(), detections[n].Height());
                    LogVerbose("tracking  ID %i  status=%i  frames=%i  lost=%i\n", detections[n].TrackID, detections[n].TrackStatus, detections[n].TrackFrames, detections[n].TrackLost);
                    LogVerbose("Object %i Edges (Left,Right,Top,Bottom)=(%.2f, %.2f, %.2f, %.2f)\n", n, detections[n].Left, detections[n].Right, detections[n].Top, detections[n].Bottom);
                    LogVerbose("video width, video height: (%.2f, %.2f)\n", input_video_width, input_video_height);
                    LogVerbose("box width, box height: (%.2f, %.2f)\n", detections[n].Width(), detections[n].Height());
                }
            }
        }
    }

#elif _WIN32



#else



#endif // JETSON_B01
}

/********************************************************************************
 * Function: print_print_performance_statsobject_info
 * Description: Print info about detection network performance.
 ********************************************************************************/
void Detection::print_performance_stats(void)
{
#ifdef JETSON_B01

    net->PrintProfilerTimes();

#elif _WIN32



#else



#endif // JETSON_B01
}

/********************************************************************************
 * Function: delete_tracking_net
 * Description: Delete detection network to free up resources.
 ********************************************************************************/
void Detection::delete_tracking_net(void)
{
#ifdef JETSON_B01

    SAFE_DELETE(net);

#elif _WIN32



#else



#endif // JETSON_B01
}

/********************************************************************************
 * Function: usage
 * Description: Display a help message.
 ********************************************************************************/
int Detection::print_usage(void)
{
#ifdef JETSON_B01

    PrintPass::c_printf("usage: detectnet [--help] [--network=NETWORK] [--threshold=THRESHOLD] ...\n");
    PrintPass::c_printf("                 input [output]\n\n");
    PrintPass::c_printf("Locate objects in a video/image stream using an object detection DNN.\n");
    PrintPass::c_printf("See below for additional arguments that may not be shown above.\n\n");
    PrintPass::c_printf("positional arguments:\n");
    PrintPass::c_printf("    input           resource URI of input stream  (see videoSource below)\n");
    PrintPass::c_printf("    output          resource URI of output stream (see videoOutput below)\n\n");

    PrintPass::c_printf("%s", detectNet::Usage());
    PrintPass::c_printf("%s", objectTracker::Usage());
    PrintPass::c_printf("%s", videoSource::Usage());
    PrintPass::c_printf("%s", videoOutput::Usage());
    PrintPass::c_printf("%s", Log::Usage());

#elif _WIN32



#else



#endif // JETSON_B01

    return 0;
}

/********************************************************************************
 * Function: detection_loop
 * Description: Process video stream and output detected objects.
 ********************************************************************************/
void Detection::detection_loop(void)
{
#ifdef JETSON_B01

    detect_objects();
    get_object_info();

#elif _WIN32

    detections = pre_process(image, net);
    image = post_process(image, detections, class_list);

    // Put efficiency information.
    std::vector<double> layersTimes;
    double freq = cv::getTickFrequency() / 1000;
    double t = net.getPerfProfile(layersTimes) / freq;
    std::string label = cv::format("Inference time : %.2f ms", t);

    cv::putText(image, label, cv::Point(20, 40), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 1);

    // Show output.
    //cv::imshow("MyVid", image);

#else



#endif // JETSON_B01
}

/********************************************************************************
 * Function: initialize_detection_net
 * Description: Delete detection network to free up resources.
 ********************************************************************************/
bool Detection::detection_net_init(void)
{
#ifdef JETSON_B01

    net = NULL;
    detections = NULL;
    numDetections = 0;

#elif _WIN32



#else



#endif // JETSON_B01

    if (!create_detection_network())
    {
        PrintPass::c_fprintf("Failed to create detection network");
        return false;
    }

    return true;
}

/********************************************************************************
 * Function: shutdown
 * Description: Shutdown detection network
 ********************************************************************************/
void Detection::shutdown(void)
{
#ifdef JETSON_B01

    LogVerbose("detectnet:  shutting down...\n");
    Detection::delete_tracking_net();
    LogVerbose("detectnet:  shutdown complete.\n");

#elif _WIN32



#else



#endif // JETSON_B01
}

#endif // ENABLE_CV
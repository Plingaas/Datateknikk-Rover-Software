#include <opencv2/opencv.hpp>
#include <iostream>

// Function to create the GStreamer pipeline string
std::string gstreamer_pipeline(int capture_width, int capture_height, int display_width, int display_height, int framerate, int flip_method) {
    return "nvarguscamerasrc ! queue max-size-buffers=1 leaky=downstream ! video/x-raw(memory:NVMM), width=(int), height=(int), format=(string)NV12, framerate=(fraction)" + std::to_string(framerate) +
           "/1 ! nvvidconv ! video/x-raw, format=I420 ! x264enc tune=zerolatency ! h264parse ! rtph264pay ! queue ! udpsink host=127.0.0.1 port=5000 ! appsink";
}

int main() {
    // Define video parameters
    int capture_width = 1280;
    int capture_height = 720;
    int display_width = 640;
    int display_height = 480;
    int framerate = 60;
    int flip_method = 0;

    // Generate the GStreamer pipeline string
    std::string pipeline = gstreamer_pipeline(capture_width,
                                              capture_height,
                                              display_width,
                                              display_height,
                                              framerate,
                                              flip_method);

    std::cout << "Using pipeline: \n\t" << pipeline << "\n";

    // Initialize the video capture with the GStreamer pipeline
    cv::VideoCapture cap(pipeline, cv::CAP_GSTREAMER);
    if (!cap.isOpened()) {
        std::cerr << "Error: Could not open camera with the GStreamer pipeline." << std::endl;
        return -1;
    }

    std::cout << "Streaming video using H.264 over UDP..." << std::endl;
    std::cout << "Press 'Esc' to stop the streaming." << std::endl;

    cv::Mat frame;
    while (true) {
        // Capture a frame (though we're streaming directly, this can be useful for debugging)
        cap >> frame;
        if (frame.empty()) break;

        // Display the frame locally (for debugging purposes)
        cv::imshow("Camera Stream", frame);
        if (cv::waitKey(10) == 27) break; // Press 'Esc' to exit the loop
    }

    // Release resources
    cap.release();
    cv::destroyAllWindows();
    std::cout << "Streaming stopped." << std::endl;
    return 0;
}

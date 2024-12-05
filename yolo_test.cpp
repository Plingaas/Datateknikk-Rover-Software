#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <fstream>
int main() {



    // Load YOLO config and weights
    std::string windowName{"YOLO Object Detection"};
    namedWindow(windowName);
    Mat frame, blob;
    while (true) {

        if (!cap.read(frame)) {
            continue;
        }
        // Create a 4D blob from the frame

        // Display the frame
        imshow(windowName, frame);
        // Break if the user presses 'q'
        const auto key = waitKey(1) == 'q';
        if (key) {
            break;
        }
    }
    return 0;
}
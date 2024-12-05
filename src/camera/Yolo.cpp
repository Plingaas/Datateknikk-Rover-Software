//
// Created by peter on 04.12.24.
//
#include "Yolo.hpp"

Yolo::Yolo() {
    std::thread init_thread([this] {
        modelConfiguration = "data/yolov7-tiny.cfg";
         modelWeights = "data/yolov7-tiny.weights";
         net = cv::dnn::readNetFromDarknet(modelConfiguration, modelWeights);
         net.setPreferableBackend(DNN_BACKEND_CUDA);
         net.setPreferableTarget(DNN_TARGET_CUDA);
         // Load the COCO class names
         std::string classesFile = "data/coco.names";
         classNames = getClassNames(classesFile);
         ready = true;
    });

    init_thread.detach();
}

void Yolo::detect(Mat& frame) {
    if (!ready) return;
    predictions.clear();
    Mat blob;
    blobFromImage(frame, blob, 1 / 255.0, Size(416, 416), Scalar(), true, false);
    net.setInput(blob);
    // Run forward pass
    std::vector<Mat> outs;
    net.forward(outs, getOutputNames(net));
    // Post-process detections
    std::vector<int> classIds;
    std::vector<float> confidences;
    std::vector<Rect> boxes;
    float confThreshold = 0.5;
    float nmsThreshold = 0.4;
    for (auto &out: outs) {
        auto data = (float *) out.data;
        for (int j = 0; j < out.rows; ++j, data += out.cols) {
            Mat scores = out.row(j).colRange(5, out.cols);
            Point classIdPoint;
            double confidence;
            minMaxLoc(scores, 0, &confidence, 0, &classIdPoint);
            if (confidence > confThreshold) {
                int centerX = (int) (data[0] * frame.cols);
                int centerY = (int) (data[1] * frame.rows);
                int width = (int) (data[2] * frame.cols);
                int height = (int) (data[3] * frame.rows);
                int left = centerX - width / 2;
                int top = centerY - height / 2;
                classIds.push_back(classIdPoint.x);
                confidences.emplace_back((float) confidence);
                boxes.emplace_back(left, top, width, height);
            }
        }
    }
    // Non-maximum suppression to remove redundant overlapping boxes
    std::vector<int> indices;
    NMSBoxes(boxes, confidences, confThreshold, nmsThreshold, indices);
    for (int idx: indices) {
        Rect box = boxes[idx];
        predictions.push_back(Prediction(classIds[idx],
            confidences[idx],
            box.x,
            box.y,
            box.x + box.width,
            box.y + box.height
            ));
    }
}


// Function to get class names
std::vector<std::string> Yolo::getClassNames(const std::string &classFile) {
    std::vector<std::string> classNames;
    std::ifstream ifs(classFile.c_str());
    std::string line;
    while (getline(ifs, line)) {
        classNames.push_back(line);
    }
    return classNames;
}

// Function to get YOLO output layer names
std::vector<String> Yolo::getOutputNames(const Net &net) {
    static std::vector<String> names;
    if (names.empty()) {
        // Get indices of output layers
        std::vector<int> outLayers = net.getUnconnectedOutLayers();
        // Get names of all layers in the network
        std::vector<String> layersNames = net.getLayerNames();
        // Get the names of the output layers using their indices
        names.resize(outLayers.size());
        for (size_t i = 0; i < outLayers.size(); ++i) {
            names[i] = layersNames[outLayers[i] - 1];
        }
    }
    return names;
}
// Function to draw bounding boxes
void Yolo::drawPred(Mat &frame) {
    if (!ready) return;
    for (auto pred : predictions) {
        rectangle(frame, Point(pred.left, pred.top), Point(pred.right, pred.bottom), Scalar(255, 178, 50), 3);
        std::ostringstream stream;
        stream << std::fixed << std::setprecision(2) << pred.conf;
        std::string label = stream.str();

        if (!classNames.empty()) {
            CV_Assert(pred.classId < classNames.size());
            label = classNames[pred.classId] + ": " + label;
        }
        int baseLine;
        Size labelSize = getTextSize(label, FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
        pred.top = max(pred.top, labelSize.height);
        rectangle(frame, Point(pred.left, pred.top - labelSize.height),
                  Point(pred.left + labelSize.width, pred.top + baseLine), Scalar::all(255), FILLED);
        putText(frame, label, Point(pred.left, pred.top), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 1);
    }

}
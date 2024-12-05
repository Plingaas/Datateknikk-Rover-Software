//
// Created by peter on 04.12.24.
//

#ifndef YOLO_HPP
#define YOLO_HPP

#endif //YOLO_HPP
#include <string>
#include <vector>
#include "opencv2/opencv.hpp"
#include "opencv2/dnn.hpp"
#include <fstream>
#include <atomic>
#include <thread>

using namespace cv;
using namespace dnn;

struct Prediction {
    int classId;
    float conf;
    int left;
    int top;
    int right;
    int bottom;

    Prediction(int classId_, float conf_, int left_, int top_, int right_, int bottom_) :
        classId(classId_), conf(conf_), left(left_), top(top_), right(right_), bottom(bottom_) {};

};

class Yolo {
public:
    Yolo();
    void detect(Mat& frame);
    void drawPred(Mat& frame);

private:
    std::atomic<bool> ready;
    std::vector<std::string> classNames;
    std::vector<Prediction> predictions;
    std::vector<std::string> getClassNames(const std::string &classFile);
    std::vector<String> getOutputNames(const Net &net);

    std::string modelConfiguration ;
    std::string modelWeights;
    Net net;
    std::string classesFile;
};
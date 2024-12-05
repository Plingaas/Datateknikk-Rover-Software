//
// Created by peter on 29.11.24.
//

#ifndef CAMERASTREAMER_HPP
#define CAMERASTREAMER_HPP
#include "simple_socket/UDPSocket.hpp"
#include <opencv2/opencv.hpp>
#include <future>
#include "Yolo.hpp"
#include <thread>
#define BUFFER_SIZE 1024
#define METADATA_SIZE 1
#define JPEG_ENCODING_QUALITY 30

struct Packet {
    uint8_t id;
    std::vector<uint8_t> data;

    Packet(std::vector<uint8_t>& data_) : id(0), data(data_) {};
    Packet(std::vector<uint8_t>& data_, uint8_t id_) : data(data_), id(id_) {
        addMetaData();
    };
private:
    void addMetaData() {
        data.insert(data.begin(), id);
    }
};

class CameraStreamer {

public:
    explicit CameraStreamer(int mode, int w, int h, int fps, int flip_method = 0);
    void setClient(const std::string& ip, const uint16_t port);
    void start();
    void stop();

private:
    std::unique_ptr<Yolo> yolo;
    int frameIndex;

    std::unique_ptr<simple_socket::UDPSocket> server;
    std::string clientIP;
    const uint16_t serverPort = 8552;
    uint16_t clientPort;

    cv::Mat frame;

    bool streaming;

    int cw, ch, outw, outh, fps, maxfps, flip_method;
    std::string pipeline;
    cv::VideoCapture camera;

    void setCameraMode(int mode);
    void setGSTPipeline();
    void setSafeGSTPipeline();
    bool updateFrame();
    void sendFrame();
    std::shared_ptr<std::vector<Packet>> generatePackets(std::vector<uint8_t>& data);
    void sendPackets(std::shared_ptr<std::vector<Packet>> packets);
    void sendPacket(const Packet& packet);
};

#endif //CAMERASTREAMER_HPP

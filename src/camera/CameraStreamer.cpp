//
// Created by peter on 29.11.24.
//
#include <CameraStreamer.hpp>

CameraStreamer::CameraStreamer(int mode_, int w_, int h_, int fps_, int flip_method_) {

    setCameraMode(mode_);
    fps = std::min(fps_, maxfps);
    outw = w_;
    outh = h_;
    flip_method = flip_method_;
    server = std::make_unique<simple_socket::UDPSocket>(serverPort);

    streaming = false;
}

void CameraStreamer::setCameraMode(int mode) {
    switch (mode) {
        case 0:
            cw = 3280;
            ch = 2464;
            maxfps = 21;
            break;
        case 1:
            cw = 3280;
            ch = 1848;
            maxfps = 28;
            break;

        case 2:
            cw = 1920;
            ch = 1080;
            maxfps = 30;
            break;

        case 3:
            cw = 1640;
            ch = 1232;
            maxfps = 30;
            break;

        case 4:
            cw = 1280;
            ch = 720;
            maxfps = 60;
            break;

        default:
            setCameraMode(1);
    }
}

void CameraStreamer::setGSTPipeline() {
    pipeline = "nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)" + std::to_string(cw) + ", height=(int)" +
           std::to_string(ch) + ", framerate=(fraction)" + std::to_string(fps) +
           "/1 ! nvvidconv flip-method=" + std::to_string(flip_method) + " ! video/x-raw, width=(int)" + std::to_string(outw) + ", height=(int)" +
           std::to_string(outh) + ", format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";
}

void CameraStreamer::setSafeGSTPipeline() {
    pipeline = "nvarguscamerasrc ! video/x-raw(memory:NVMM), width=1280, height=720, framerate=60/1 ! nvvidconv ! video/x-raw, width=(int)" + std::to_string(outw) + ", height=(int)" +
           std::to_string(outh) + ", format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";
}

bool CameraStreamer::updateFrame() {
    return camera.read(frame);
}

void CameraStreamer::sendFrame() {

    std::vector<uint8_t> data;
    cv::imencode(".jpg", frame, data, {cv::IMWRITE_JPEG_QUALITY, JPEG_ENCODING_QUALITY});
    auto packets = generatePackets(data);
    sendPackets(packets);
}

std::shared_ptr<std::vector<Packet>> CameraStreamer::generatePackets(std::vector<uint8_t>& data) {

    // Bytes to send
    uint32_t bytesRemaining = data.size();

    // Total packets containing image data to send
    uint16_t nPackets = (bytesRemaining / (BUFFER_SIZE-METADATA_SIZE)) + static_cast<uint32_t>((bytesRemaining % (BUFFER_SIZE-METADATA_SIZE)) > 0);

    // Organize packets before sending
    auto packets = std::make_shared<std::vector<Packet>>();

    // Start of frame contains information about the packets to be sent
    std::vector<uint8_t> SOF = {0xd8, (nPackets >> 8) & 0xff, nPackets & 0xff, 0x8d};
    Packet SOFPacket(SOF);
    packets->push_back(SOFPacket);

    uint16_t bytesToSend;
    for (int i = 0; i < nPackets; i++) {
        if (bytesRemaining > BUFFER_SIZE - METADATA_SIZE) {
            bytesToSend = BUFFER_SIZE - METADATA_SIZE;
        } else {
            bytesToSend = bytesRemaining;
        }
        std::vector<uint8_t> metaData = {static_cast<uint8_t>(i)};
        uint32_t start = i * (BUFFER_SIZE - METADATA_SIZE);
        uint32_t end = start + bytesToSend;

        std::vector<uint8_t> packetData;
        packetData.insert(packetData.begin(), data.begin() + start, data.begin() + end);
        Packet packet(packetData, i);
        packets->push_back(packet);

        bytesRemaining-=bytesToSend;
    }

    return packets;
}

void CameraStreamer::sendPackets(std::shared_ptr<std::vector<Packet>> packets) {
    for (auto packet : (*packets)) {
        server->sendTo(clientIP, clientPort, packet.data);
    }
}

void CameraStreamer::setClient(const std::string& ip, const uint16_t port) {
    clientIP = ip;
    clientPort = port;
}

void CameraStreamer::start() {
    setGSTPipeline();
    try {
        camera = cv::VideoCapture(pipeline, cv::CAP_GSTREAMER);
    } catch (cv::Exception e) {
        std::cout << "Unable to open camera with given pipeline. " << e.what() << std::endl;
        try {
            setSafeGSTPipeline();
            camera = cv::VideoCapture(pipeline, cv::CAP_GSTREAMER);
        } catch (cv::Exception e) {
            std::cout << e.what() << std::endl;
        }
    }
    std::thread stream_thread([this]() {
       while (camera.isOpened()) {
           try {
               updateFrame();
               sendFrame();
           } catch (cv::Exception& e) {
               std::cout << e.what() << std::endl;
           } catch (std::exception& e) {
               std::cout << e.what() << std::endl;
           }
       }
    });

    stream_thread.detach();
}

void CameraStreamer::stop() {
    camera.release();
}
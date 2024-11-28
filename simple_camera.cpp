// simple_camera.cpp
// MIT License
// Copyright (c) 2019-2022 JetsonHacks
// See LICENSE for OpenCV license and additional information
// Using a CSI camera (such as the Raspberry Pi Version 2) connected to a 
// NVIDIA Jetson Nano Developer Kit using OpenCV
// Drivers for the camera and OpenCV are included in the base image

#include <opencv2/opencv.hpp>
#include <opencv2/cudacodec.hpp>
#include <iostream>
#include <cstring>
#include <thread>
#include <simple_socket/UDPSocket.hpp>
#include <future>

#define BUFFER_SIZE 1024

std::string gstreamer_pipeline (int capture_width, int capture_height, int display_width, int display_height, int framerate, int flip_method) {
    return "nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)" + std::to_string(capture_width) + ", height=(int)" +
           std::to_string(capture_height) + ", framerate=(fraction)" + std::to_string(framerate) +
           "/1 ! nvvidconv flip-method=" + std::to_string(flip_method) + " ! video/x-raw, width=(int)" + std::to_string(display_width) + ", height=(int)" +
           std::to_string(display_height) + ", format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";
}
void printbuffffer(std::vector<uint8_t> buffer, int elements) {
	std::ostringstream oss;
	for (int i = 0;i<elements;i++) {
		oss << "0x" << std::setw(2) << std::setfill('0') <<  std::hex <<  (int)(buffer[i]) << " ";
	}
	std::cout << oss.str() << std::endl;
}

long getCurrentTime() {
	auto now = std::chrono::system_clock::now();
	auto duration = now.time_since_epoch();
	auto t = std::chrono::duration_cast<std::chrono::microseconds>(duration).count();
	return t;
}

bool sendAndReceive(std::unique_ptr<simple_socket::UDPSocket> server, const std::vector<uint8_t>& data) {
	std::string ip("10.24.43.131");
	uint16_t port = 8553;
	std::vector<uint8_t> buf = {0x01, 0x02, 0x03};
	server->sendTo(ip, port, buf);

	std::vector<uint8_t> response;
	std::cout << "Waiting for response" << std::endl;
	server->recvFrom(ip, port, response);
	std::cout << "Received response" << std::endl;
	return response[0] == 1;
}

int main()
{
	//auto server = std::make_unique<simple_socket::TCPServer>(8552);
	//auto conn = server->accept();
	auto server = std::make_unique<simple_socket::UDPSocket>(8552);
    int capture_width = 3280 ;
    int capture_height = 1848 ;
    int display_width = 640 ;
    int display_height = 480 ;
    int framerate = 28 ;
    int flip_method = 0 ;

    std::string pipeline = gstreamer_pipeline(capture_width,
	capture_height,
	display_width,
	display_height,
	framerate,
	flip_method);
    std::cout << "Using pipeline: \n\t" << pipeline << "\n";
 
    cv::VideoCapture cap(pipeline, cv::CAP_GSTREAMER);
	while(!cap.isOpened()) {
		std::cout<<"Failed to open camera."<<std::endl;
	}

    cv::Mat img;

    while(true) {
		std::vector<uint8_t> data;
    	if (!cap.read(img)) {
			std::cout<<"Capture read error"<<std::endl;
			continue;
		}
    	cv::imshow("img", img);
    	cv::waitKey(1);
		cv::imencode(".jpg", img, data);

		uint32_t bytesRemaining = data.size();
		uint8_t metaDataSize = 3;
    	uint32_t nPackets = (bytesRemaining / (BUFFER_SIZE-metaDataSize)) + static_cast<uint32_t>((bytesRemaining % (BUFFER_SIZE-metaDataSize)) > 0);
    	std::vector<uint8_t> startBytes = {0xd8, (nPackets >> 8) & 0xff, nPackets & 0xff, 0x8d};
    	// Organize packets before sending
    	std::vector<std::vector<uint8_t>> packets(nPackets);
    	uint16_t bytes;
    	for (int i = 0; i < nPackets; i++) {
    		if (bytesRemaining > BUFFER_SIZE - metaDataSize) {
				bytes = BUFFER_SIZE - metaDataSize;
    		} else {
    			bytes = bytesRemaining;
    		}
    		std::vector<uint8_t> metaData = {static_cast<uint8_t>(i), static_cast<uint8_t>((bytes >> 8) & 0xff), static_cast<uint8_t>(bytes & 0xff)};
    		uint32_t start = i * (BUFFER_SIZE - metaDataSize);
    		uint32_t end = start + bytes;

    		packets[i].insert(packets[i].begin(), data.begin() + start, data.begin() + end); // Frame data
    		packets[i].insert(packets[i].begin(), metaData.begin(), metaData.end()); // Frame index
    		bytesRemaining-=BUFFER_SIZE;
    	}

    	// Attempt to send data
    	bool sending = true;
    	auto timeout = std::chrono::milliseconds(100);
    	std::string ip("10.24.43.131");
    	uint16_t port = 8553;
		uint8_t attempts = 0;
    	std::cout << "Start sending" << std::endl;
    	while (sending) {
    		server->sendTo(ip, port, startBytes); // Send startbytes
    		for (int i = 0; i < nPackets;) {
    			std::future<bool> result = std::async(std::launch::async, sendAndReceive, std::move(server), data);
    			auto status = result.wait_for(timeout);
    			sending = static_cast<bool>(status);
    			if (!result.get()) {
    				attempts++;
    				std::cout << "No response" << std::endl;
    				if (attempts == 5) {
    					sending = false;
    					break;
    				}
    			} else {
    				std::cout << "Packet sent successfully" << std::endl;
    				i++;
    				attempts = 0;
    			}
    		}
    	}




		/*
		uint32_t data_size = data.size();
    	std::cout << data_size << std::endl;
    	std::vector<uint8_t> dataSize;
    	for (size_t i = 0; i < sizeof(data_size); ++i) {
    		dataSize.push_back(static_cast<uint8_t>((data_size >> (i*8)) & 0xFF));
    	}

    	std::vector<uint8_t> startBytes = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};

    	int packetSize = BUFFER_SIZE + 1;
    	int totalBytes =
    	int minsize = BUFFER_SIZE-12;
    	size_t packets = (data_size / BUFFER_SIZE) + (data_size % BUFFER_SIZE) > minsize + 1;

    	data.insert(data.begin(), startBytes.begin(), startBytes.end());
    	data.insert(data.begin()+7, static_cast<uint8_t>(packets));

    	data.insert(data.begin() + 8, dataSize.begin(), dataSize.end());

    	for (int i = 0; i < packets; i++) {
    		std::vector<uint8_t> packet(BUFFER_SIZE,0);
    		uint32_t start = i * BUFFER_SIZE;
    		uint32_t end = std::min(start + BUFFER_SIZE, data_size);
    		std::memcpy(packet.data(), &data[start], end - start);
    		///conn->write(packet);
    		server->sendTo("10.24.41.131", 8553, packet);
    	}
		data.erase(data.begin(), data.begin()+12);
    	auto newimg = cv::imdecode(data, cv::IMREAD_COLOR);
		cv::imshow("img", newimg);
    	cv::waitKey(1);
    	*/
    }

    cap.release();
    cv::destroyAllWindows() ;
    return 0;
}



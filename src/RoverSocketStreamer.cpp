//
// Created by peter on 04.11.24.
//

#include "RoverStateStreamer.hpp"
#include <cstring>

namespace Rover {
    StateStreamer::StateStreamer(int port) {

        server = std::make_unique<TCPServer>(port);

        sendThread = std::thread([this] {
            send();
        });

        sendThread.detach();
    };

    void StateStreamer::handleConnection(std::unique_ptr<SimpleConnection> conn) {
        std::vector<uint8_t> buffer(256);
        const auto bytesRead = conn->read(buffer);
        buffer.erase(buffer.begin() + bytesRead, buffer.end());
        inPackets.push(buffer);
        std::string data(buffer.begin(), buffer.begin() + bytesRead);
        std::cout << "Received: " << data << std::endl;
    };

    void StateStreamer::listen() {

        try {
            while (true) {
                //std::unique_ptr<SimpleConnection> conn = server->accept();
                //handleConnection(std::move(conn));
            }
        } catch (std::exception &e) {
            std::cerr << e.what() << std::endl;
        }
    };
    void StateStreamer::send() {
        while (true) {
            try {

                client = server->accept();
                std::cout << "Client connected" << std::endl;
                bool connected = true;
                while (true) {
                    if (!connected) { // This doesnt work yet need to figure out why
                        client = server->accept();
                    }
                    if (!outPackets.empty()) {
                        try {
                            client->write(outPackets.front());
                        } catch (std::exception &e) {
                            std::cout << "Client disconnected? | " << e.what() << std::endl;
                            connected = false;
                        }
                        outPackets.pop();
                    }
                }
            } catch (std::exception &e) {
                std::cout << e.what() << std::endl;
            }
        }
    }
}
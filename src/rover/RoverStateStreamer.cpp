//
// Created by peter on 04.11.24.
//

#include "RoverStateStreamer.hpp"
#include <cstring>
#include <mutex>

namespace Rover {
    StateStreamer::StateStreamer(int port) {
        
        server = std::make_unique<TCPServer>(port);

        sendThread = std::thread([this] {
            stream();
        });

        sendThread.detach();
    };

    void StateStreamer::stream() {
        while (true) {
            std::cout << "Waiting for client to connect" << std::endl;
            client = server->accept();
            std::cout << "Client connected" << std::endl;
            bool connected = true;
            while (connected) {
                if (outPackets.empty()) continue;
                connected = sendPacket();
            }
            std::cout << "Client disconnected" << std::endl;
        }
    }

    bool StateStreamer::sendPacket() {
        std::mutex m;
        std::lock_guard<std::mutex> lock(m);
        bool success = client->write(outPackets.front());
        if (success) {
            outPackets.pop();
        }
        return success;
    }
}

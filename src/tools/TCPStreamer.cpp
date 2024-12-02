//
// Created by peter on 01.12.24.
//

#include "TCPStreamer.hpp"

TCPStreamer::TCPStreamer(uint16_t port_, bool verbose_) {
    port = port_;
    server = std::make_unique<simple_socket::TCPServer>(port);
    connected = false;
    verbose = verbose_;
    timeout = 0;
}

void TCPStreamer::waitForConnection() {
    std::thread connection_thread([this] {
        if (verbose) std::cout << "Listening for client connection." << std::endl;
        client = server->accept();

        std::lock_guard<std::mutex> lock(m);
        connected = true;

        if (connect_handler) {
            connect_handler();
        }
    });

    connection_thread.detach();
}

void TCPStreamer::startStreaming() {
    std::thread streaming_thread([this] {
        waitForConnection();
        while (true) {
            while (connected) {
                if (packets.empty()) continue;
                connected = sendPacket();
                if (connected) continue;

                time_of_disconnect = getCurrentTime();
                if (verbose) std::cout << "Client disconnected... Attempting to reconnect." << std::endl;
                waitForConnection(); // Attempt to reconnect

                // Wait for reconnection
                bool reconnected = true;
                while (!connected) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Offload the cpu.
                    if (getCurrentTime() - time_of_disconnect > timeout) {
                        if (disconnect_handler) { // Run disconnect handler if set.
                            disconnect_handler();
                        }
                        reconnected = false;
                        if (verbose) std::cout << "Failed to reconnect." << std::endl;
                        break;
                    }
                }

                if (reconnected && reconnect_handler) {
                    reconnect_handler();
                }
            }
        }
    });
    streaming_thread.detach();
}

bool TCPStreamer::sendPacket() {
    bool success = client->write(packets.front());
    if (success) {
        //packets.pop();
    }
    return success;
}

void TCPStreamer::addPacket(std::vector<uint8_t> &packet) {
    std::lock_guard<std::mutex> lock(m);
    packets.push(packet);
}

long TCPStreamer::getCurrentTime() {
    auto now = std::chrono::system_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch());
    return duration.count();
}

void TCPStreamer::close() {

}
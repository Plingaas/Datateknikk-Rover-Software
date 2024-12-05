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
    SOF_ = {0xd8, 0xd9, 0xda};
}

void TCPStreamer::waitForConnection() {
    std::thread connection_thread([this] {
        if (verbose) printMessage("Listening for client connection.");
        client = server->accept();

        std::lock_guard<std::mutex> lock(m);
        connected = true;

        if (connect_handler) {
            if (verbose) printMessage("Client connected.");
            connect_handler();
        }
    });

    connection_thread.join();
}

void TCPStreamer::startStreaming() {
    std::thread streaming_thread([this] {
        while (true) {
            waitForConnection();
            while (connected) {
                if (packets.empty()) continue;
                connected = sendPacket();
                if (connected) continue;

                uint64_t time_of_disconnect = getCurrentTime();
                if (verbose) printMessage("Lost connection to client.");

                std::thread reconnect_thread([this] {
                    waitForConnection(); // Attempt to reconnect
                });
                reconnect_thread.detach(); // Detach so we can check for disconnect timeout reached.

                // Wait for reconnection
                bool reconnected = true;
                while (!connected) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Offload the cpu.
                    if (getCurrentTime() - time_of_disconnect > timeout) {
                        if (verbose) printMessage("Failed to reconnect.");
                        if (disconnect_handler) disconnect_handler();
                        reconnected = false;
                        break;
                    }
                }

                if (reconnected && reconnect_handler) {
                    if (verbose) printMessage("Client reconnected.");
                    reconnect_handler();
                }
            }
        }
    });
    streaming_thread.detach();
}

bool TCPStreamer::sendPacket() {
    std::lock_guard<std::mutex> lock(m);
    bool success = client->write(packets.front());
    if (success) {
        packets.pop();
    }
    return success;
}

void TCPStreamer::addPacket(std::vector<uint8_t> &packet) {
    // Insert unix time
    std::vector<uint8_t> t_bytes(sizeof(uint64_t));
    getCurrentTimeBytes(t_bytes);

    std::lock_guard<std::mutex> lock(m);
    packet.insert(packet.begin(), t_bytes.begin(), t_bytes.end());
    packet.insert(packet.begin(), SOF_.begin(), SOF_.end()); // Insert SOF
    packets.push(packet);
}

void TCPStreamer::getCurrentTimeBytes(std::vector<uint8_t>& buffer) {
    auto now = std::chrono::system_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch());
    uint64_t t = duration.count();
    std::memcpy(buffer.data(), &t, sizeof(uint64_t));
}

uint64_t TCPStreamer::getCurrentTime() {
    auto now = std::chrono::system_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch());
    return duration.count();
}

void TCPStreamer::close() {

}

void TCPStreamer::printMessage(std::string message) {
    std::cout << "(" << name << "): " << message << std::endl;
}
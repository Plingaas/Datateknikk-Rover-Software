//
// Created by peter on 01.12.24.
//

#ifndef TCPSTREAMER_HPP
#define TCPSTREAMER_HPP
#include <functional>

#include "simple_socket/TCPSocket.hpp"
#include <queue>
#include <iostream>
#include <thread>
#include <mutex>
#include <chrono>
#include <condition_variable>
#include <atomic>

class TCPStreamer {

public:
    using DisconnectHandler = std::function<void()>;
    using ConnectHandler = std::function<void()>;
    using ReconnectHandler = std::function<void()>;

    void setDisconnectHandler(DisconnectHandler handler) {
        disconnect_handler = std::move(handler);
    }
    void setConnectHandler(ConnectHandler handler) {
        connect_handler = std::move(handler);
    }
    void setReconnectHandler(ConnectHandler handler) {
        reconnect_handler = std::move(handler);
    }

    explicit TCPStreamer(uint16_t port_, bool verbose_ = false);
    void addPacket(std::vector<uint8_t>& packet);
    bool sendPacket();
    void startStreaming();
    void setTimeout(long timeout_) {timeout = timeout_;};
    void setPort(uint16_t port_) {port = port_;};
    void close();
    void setName(std::string name_) {name = name_;};

private:
    void printMessage(std::string message);
    std::string name;
    bool verbose;
    std::unique_ptr<simple_socket::TCPServer> server;
    std::unique_ptr<simple_socket::SimpleConnection> client;
    std::mutex m;
    std::queue<std::vector<uint8_t>> packets;

    uint16_t port;

    long timeout;
    long time_of_disconnect;
    std::atomic<bool> connected;

    long getCurrentTime();
    void waitForConnection();

    DisconnectHandler disconnect_handler;
    ConnectHandler connect_handler;
    ReconnectHandler reconnect_handler;
};
#endif //TCPSTREAMER_HPP

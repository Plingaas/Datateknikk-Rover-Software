//
// Created by peter on 04.11.24.
//


#ifndef ROVERSOCKETSTREAMER_HPP
#define ROVERSOCKETSTREAMER_HPP
#include "simple_socket/TCPSocket.hpp"
#include "simple_socket/UDPSocket.hpp"
#include "simple_socket/util/port_query.hpp"
#include <iostream>
#include <queue>
#include <thread>

using namespace simple_socket;

namespace Rover {

    class StateStreamer {
    private:
        std::optional<uint16_t> port;
        std::vector<uint8_t> buffer;
        std::queue<std::vector<uint8_t>> inPackets;
        std::queue<std::vector<uint8_t>> outPackets;
        std::unique_ptr<TCPServer> server;
        std::unique_ptr<SimpleConnection> client;
        std::thread serverThread;
        std::thread sendThread;

    public:
        StateStreamer(int port);
        void handleConnection(std::unique_ptr<SimpleConnection> conn);

        void addPacket(std::vector<uint8_t>& packet) {outPackets.push(packet);};
        void listen();
        void send();

    };
};
#endif //ROVERSOCKETSTREAMER_HPP

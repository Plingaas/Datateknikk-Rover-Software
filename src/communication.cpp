#include "communication.hpp"
#include "simple_socket/TCPSocket.hpp"

Communication::Communication() {
}

void Communication::startVideoServer() {
}

void Communication::sendLidarData(
    std::unique_ptr<simple_socket::SimpleConnection>& connection,
    std::queue<std::vector<std::pair<double,double>>>& coordinates_queue,
    std::mutex& m,
    std::atomic_bool& stopper)
{

    while (true) {
        while (!stopper) {
            if (coordinates_queue.empty()) continue;

            /*cv.wait(lock, [&stopper, &coordinates_queue] {
                return stopper || !coordinates_queue.empty();
            });*/
            std::unique_lock<std::mutex> lock(m);
            auto data = coordinates_queue.front();//makes a copy of the coordinates
            coordinates_queue.pop();
            lock.unlock(); //unlocks the coordinates

            auto buffer = serializeLidarData(data);

            connection->write(buffer);
        }
    }

}

void Communication::Receivecommands(std::unique_ptr<simple_socket::SimpleConnection>& connection) {

    //Buffer for commands
    std::vector<unsigned char> buffer(1024);

    connection->read(buffer);

}


/*std::vector<unsigned char> Communication::serializeLidarData(std::vector<std::pair<double,double>>& data) {

    std::vector<unsigned char> buffer(data.size()* sizeof(std::pair<double, double>));

    memcpy(buffer.data(), data.data(), buffer.size());

    return buffer;

}*/
#include <iostream>
std::vector<unsigned char> Communication::serializeLidarData(const std::vector<std::pair<double, double>>& data) {
    std::vector<unsigned char> buffer(8192);
    //buffer.reserve(data.size() * 2 * sizeof(double)); // Reserve enough space for pairs of doubles
    int index = 0;
    for (const auto& pair : data) {
        std::vector<uint8_t> bytes(sizeof(pair));
        std::memcpy(bytes.data(), &pair, sizeof(pair)); // Copy each double into bytes
        buffer.insert(buffer.begin() + index, bytes.begin(), bytes.end());
        index += sizeof(pair);
    }

    return buffer;
}




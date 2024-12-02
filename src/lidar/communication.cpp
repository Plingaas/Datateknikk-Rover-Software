#include "communication.hpp"
#include "simple_socket/TCPSocket.hpp"

Communication::Communication() {
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

            std::unique_lock<std::mutex> lock(m);
            auto data = coordinates_queue.front();//makes a copy of the coordinates
            coordinates_queue.pop();
            lock.unlock(); //unlocks the coordinates

            auto buffer = serializeLidarData(data);

            connection->write(buffer);
        }
    }

}



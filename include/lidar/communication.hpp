#ifndef COMMUNICATION_HPP
#define COMMUNICATION_HPP

#include <vector>
#include "simple_socket/SimpleConnection.hpp"
#include <mutex>
#include <memory>
#include <atomic>
#include <condition_variable>
#include <cstring>
#include <queue>
#include <iostream>

class Communication{

  public:

  Communication();

  void sendLidarData(std::unique_ptr<simple_socket::SimpleConnection>& connection,std::queue<std::vector<std::pair<double,double>>>& coordinates_queue,std::mutex& m,std::atomic_bool& stopper);

  std::vector<unsigned char> serializeLidarData(const std::vector<std::pair<double, double>>& data);




  private:







};

#endif //COMMUNICATION_HPP

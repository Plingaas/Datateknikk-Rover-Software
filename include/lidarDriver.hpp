#ifndef LIDARDRIVER_HPP
#define LIDARDRIVER_HPP

#include <vector>
#include "serial/serial.h"
#include <mutex>
#include <condition_variable>
#include "simple_socket/TCPSocket.hpp"
#include <queue>

class LidarDriver {

private:
  std::unique_ptr<serial::Serial> device;
  std::vector<unsigned char> stop_ = {0xA5, 0xF0, 0x02, 0x00, 0x00};
  std::vector<unsigned char> stopBytes = checkSum(stop_);

  std::vector<std::pair<double, double>> cartesianCoordinates_;
  std::vector<std::pair<double, double>> polarCoordinates_;

  public:

  LidarDriver();

  void startMotorHalf();
  void stopMotor();
  const std::vector<uint8_t> checkSum(const std::vector<unsigned char>& bytes);

  void startScan1(
    std::atomic<bool>& stopper,
    std::queue<std::vector<std::pair<double,double>>> & coordinates_queue,
    std::mutex& m);

  void startScan2(std::atomic<bool>& stopper);

  void startExpressScan(bool& stopper);
  void stopScan();

  void reset();

  std::vector<unsigned char> serializeLidarData();


  std::vector<std::pair<double, double>> getCoordinates();

  void startMotorFull();
  void close();

  ~LidarDriver() {stopScan(); stopMotor();}
};



#endif //LIDARDRIVER_HPP

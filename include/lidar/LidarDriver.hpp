#ifndef LIDARDRIVER_HPP
#define LIDARDRIVER_HPP

#include <vector>
#include "serial/serial.h"
#include <mutex>
#include <condition_variable>
#include "simple_socket/TCPSocket.hpp"
#include <queue>
#include <functional>

class LidarDriver {

public:
  using ConnectHandler = std::function<void()>;
  using CloseHandler = std::function<void()>;
  using NewFrameHandler = std::function<void(std::vector<uint8_t>& data)>;

  void setConnectHandler(ConnectHandler handler) {
    connect_handler = std::move(handler);
  }
  void setCloseHandler(CloseHandler handler) {
    close_handler = std::move(handler);
  }
  void setNewFrameHandler(NewFrameHandler handler) {
    new_frame_handler = std::move(handler);
  }

  LidarDriver();
  void stopScan();
  void stopMotor();

  void startScan(std::queue<std::vector<std::pair<double, double>>>* coordinates_queue);
  void startMotorHalf();
  void startMotorFull();

  void close();
  void reset();
  ~LidarDriver() {stopScan(); stopMotor();}

private:
  ConnectHandler connect_handler;
  CloseHandler close_handler;
  NewFrameHandler new_frame_handler;

  bool scanning;
  std::unique_ptr<serial::Serial> device;
  std::vector<uint8_t> serializeLidarData(const std::vector<std::pair<double, double>>& data);
  const std::vector<uint8_t> checksum(const std::vector<unsigned char>& bytes);
};



#endif //LIDARDRIVER_HPP

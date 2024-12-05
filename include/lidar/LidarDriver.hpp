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

  void start();

  void stop();
  void close();
  void reset();
  ~LidarDriver() {stopScan(); stopMotor();}
  void startMotorHalf();
  void startMotorFull();


private:
  void stopScan();
  void stopMotor();
  void startScan();
  void startExpressScan();
  void calcAndAddPoints(std::vector<std::pair<float, float>>& points,const std::vector<float>& dist1, const std::vector<float>& dist2,
                        const std::vector<float>& delta1, const std::vector<float>& delta2,
                        float previousAngle, float refAngle);
  std::tuple<uint16_t, uint16_t, int> readCabin1or3(uint8_t data);
  uint16_t readCabin2or4(uint8_t data);
  std::pair<uint16_t, uint16_t> readCabin5(uint8_t data);
  float calcDist(uint16_t distP1, uint16_t distP2);
  float calcDelta(uint16_t deltaP1, uint16_t deltaP2, int deltaSign);

  ConnectHandler connect_handler;
  CloseHandler close_handler;
  NewFrameHandler new_frame_handler;

  bool scanning;
  std::unique_ptr<serial::Serial> device;
  void serializeLidarData(const std::vector<std::pair<float, float>>& data, std::vector<uint8_t>& buffer);
  const std::vector<uint8_t> checksum(const std::vector<unsigned char>& bytes);
};



#endif //LIDARDRIVER_HPP

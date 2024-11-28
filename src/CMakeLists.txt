
add_library(rover Rover.cpp RoverSerial.cpp RoverSerialMessage.cpp RoverSerialParser.cpp RoverSerialHeader.cpp RoverSerialSensorStreamService.cpp RoverSocketStreamer.cpp)
add_library(lidar lidarDriver.cpp communication.cpp)

target_include_directories(rover PUBLIC "${CMAKE_SOURCE_DIR}/include")
target_include_directories(lidar PUBLIC "${CMAKE_SOURCE_DIR}/include")


include(FetchContent)
set(SIMPLE_SOCKET_BUILD_TESTS OFF)
FetchContent_Declare(
        SimpleSocket
        GIT_REPOSITORY https://github.com/markaren/SimpleSocket.git
        GIT_TAG master
)
FetchContent_MakeAvailable(SimpleSocket)

set(SERIAL_BUILD_EXAMPLES OFF)
FetchContent_Declare(
        serial
        GIT_REPOSITORY https://github.com/markaren/serial.git
        GIT_TAG fd4ef255f693679f0d589df2145176b7c6eccb1c
)

FetchContent_MakeAvailable(serial)


target_link_libraries(rover PUBLIC serial simple_socket)
target_link_libraries(lidar PUBLIC simple_socket serial)
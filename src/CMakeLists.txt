set(publicHeaderDir ${PROJECT_SOURCE_DIR}/include)

set(publicHeaders
    "rover/Rover.hpp"
    "lidar/Lidar.hpp"
    "camera/CameraStreamer.hpp"
)

set(privateHeaders
    "rover/RoverSerial.hpp"
    "rover/RoverSerialHeader.hpp"
    "rover/RoverSerialMessage.hpp"
    "rover/RoverSerialParser.hpp"
    "rover/RoverSerialSensorStreamService.hpp"
    "rover/RoverStateStreamer.hpp"
    "rover/RoverSensorData.hpp"
    "rover/SerialEnum.hpp"
    "lidar/LidarDriver.hpp"
    "lidar/communication.hpp"
    "tools/TCPStreamer.hpp"
)

set(sources
    "rover/Rover.cpp"
    "rover/RoverSerial.cpp"
    "rover/RoverSerialHeader.cpp"
    "rover/RoverSerialMessage.cpp"
    "rover/RoverStateStreamer.cpp"
    "rover/RoverSerialParser.cpp"
    "rover/RoverSerialSensorStreamService.cpp"
    "lidar/Lidar.cpp"
    "lidar/LidarDriver.cpp"
    "lidar/communication.cpp"
    "camera/CameraStreamer.cpp"
    "tools/TCPStreamer.cpp"
)

set(publicHeadersFull)
foreach (header IN LISTS publicHeaders)
    list(APPEND publicHeadersFull "${publicHeaderDir}/${header}")
endforeach ()

set(privateHeadersFull)
foreach (header IN LISTS privateHeaders)
    list(APPEND privateHeadersFull "${publicHeaderDir}/${header}")
endforeach ()

# OPENCV and CUDA
find_package(OpenCV REQUIRED)
find_package(CUDA REQUIRED)

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

add_library(SpheroRVRExtension ${sources} ${publicHeadersFull} ${privateHeadersFull})
target_include_directories(SpheroRVRExtension PUBLIC ${publicHeaderDir}/rover ${publicHeaderDir}/lidar ${publicHeaderDir}/camera ${publicHeaderDir}/tools)
target_link_libraries(SpheroRVRExtension PUBLIC serial simple_socket ${OpenCV_LIBS} ${CUDA_LIBRARIES})

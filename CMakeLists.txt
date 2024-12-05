cmake_minimum_required(VERSION 3.10)
project(SLAM)

set(CMAKE_CXX_STANDARD 20)


# Executables
add_executable(BasicCam simple_camera.cpp
        include/camera/Yolo.hpp)
add_executable(VideoStream cam.cpp)
add_executable(Yolo yolo_test.cpp)
add_executable(socketTest testSocket.cpp)
add_executable(testbytereading Test_byte_reading.cpp)
add_executable(main main.cpp)
add_subdirectory(src)

target_link_libraries(BasicCam PUBLIC SpheroRVRExtension)
target_link_libraries(VideoStream PUBLIC SpheroRVRExtension)
target_link_libraries(Yolo PUBLIC SpheroRVRExtension)
target_link_libraries(socketTest PUBLIC SpheroRVRExtension)
target_link_libraries(testbytereading PUBLIC SpheroRVRExtension)
target_link_libraries(main PUBLIC SpheroRVRExtension)

file(COPY "data/" DESTINATION "${CMAKE_BINARY_DIR}/data")

cmake_minimum_required(VERSION 3.10)
project(SLAM)

set(CMAKE_CXX_STANDARD 20)

# OPENCV and CUDA
find_package(OpenCV REQUIRED)
find_package(CUDA REQUIRED)

# Executables
add_executable(BasicCam simple_camera.cpp)
add_executable(VideoStream cam.cpp)
add_executable(Yolo yolo_test.cpp)
add_executable(Rover rvr.cpp)
add_executable(socketTest testSocket.cpp)
add_executable(tcp TCP.cpp)
add_executable(lidarSelf mainLidar.cpp)
add_executable(testbytereading Test_byte_reading.cpp)
add_executable(main main.cpp)

add_subdirectory(src)

target_link_libraries(BasicCam ${OpenCV_LIBS} ${CUDA_LIBRARIES})
target_link_libraries(BasicCam rover)

target_link_libraries(VideoStream ${OpenCV_LIBS} ${CUDA_LIBRARIES})
target_link_libraries(Yolo ${OpenCV_LIBS} ${CUDA_LIBRARIES})
target_link_libraries(Rover PUBLIC rover)
target_link_libraries(socketTest PUBLIC rover)
target_link_libraries(tcp PUBLIC lidar)
target_link_libraries(lidarSelf PUBLIC lidar)
target_link_libraries(testbytereading PUBLIC lidar)
target_link_libraries(main PUBLIC lidar rover)

file(COPY "data/" DESTINATION "${CMAKE_BINARY_DIR}/data")

cmake_minimum_required(VERSION 3.8)
project(rcpcc)
add_compile_options(-w)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(rosidl_default_generators REQUIRED)
# find_package(rcpcc REQUIRED)



set(FFTW_INCLUDE_DIR /usr/local/include)
set(FFTW_LIBRARY libfftw3.so)
set(ZSTD_LIBRARY libzstd.so)


rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CompressedPointCloud.msg"
  DEPENDENCIES std_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

include_directories(
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${FFTW_INCLUDE_DIR}
)
include_directories(./rcpcc/src/utils)
include_directories(./rcpcc/src/modules)
include_directories(./rcpcc/src/proto)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


set(SOURCES
    rcpcc/src/utils/utils.cpp
    rcpcc/src/utils/encoder.cpp
    rcpcc/src/utils/pcc_module.cpp
    rcpcc/src/utils/decoder.cpp
    rcpcc/src/utils/io.cpp
    rcpcc/src/modules/decoder_module.cpp
    rcpcc/src/modules/encoder_module.cpp
    rcpcc/src/modules/dct.cpp
    rcpcc/src/modules/binary_compressor.cpp
    rcpcc/src/modules/serializer.cpp
)
# adding compress executable
add_executable(rcpcc_compress ${SOURCES} src/rcpcc_compress.cpp)
ament_target_dependencies(rcpcc_compress
                          rclcpp
                          std_msgs
                          sensor_msgs
                          pcl_conversions
                          pcl_msgs)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)

target_link_libraries(rcpcc_compress ${cpp_typesupport_target})               
target_link_libraries(rcpcc_compress
  ${OpenCV_LIBRARIES}
  ${FFTW_LIBRARY}
  ${ZSTD_LIBRARY}
)


# adding decompress executable
add_executable(rcpcc_decompress ${SOURCES} src/rcpcc_decompress.cpp)
ament_target_dependencies(rcpcc_decompress
                        rclcpp
                        std_msgs
                        sensor_msgs
                        pcl_conversions
                        pcl_msgs)
rosidl_get_typesupport_target(cpp_typesupport_target2 ${PROJECT_NAME} rosidl_typesupport_cpp)

target_link_libraries(rcpcc_decompress ${cpp_typesupport_target2})
target_link_libraries(rcpcc_decompress
  ${OpenCV_LIBRARIES}
  ${FFTW_LIBRARY}
  ${ZSTD_LIBRARY}
)

install(TARGETS
        rcpcc_compress
        rcpcc_decompress
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()

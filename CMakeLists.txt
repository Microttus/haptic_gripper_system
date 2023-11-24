cmake_minimum_required(VERSION 3.8)
project(haptic_gripper_system)

# Specify the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

message(${PROJECT_NAME})

include_directories(include ../qbshr_ctr/include/qbshr_ctr ../qbshr_ctr/libs/qbdevice-api-7.x.x/serial/include ../qbshr_ctr/libs/qbdevice-api-7.x.x/qbrobotics-driver/libs/research/include)

get_filename_component(API_PATH_uq ../qbshr_ctr/libs ABSOLUTE)
if(EXISTS "${API_PATH_uq}/qbdevice-api-7.x.x/serial/CMakeLists.txt")
    message("-- [ ${PROJECT_NAME}] Serial folder found")
endif()
SET(CMAKE_PREFIX_PATH ${API_PATH_uq}/qbdevice-api-7.x.x/serial)


message(${CMAKE_CURRENT_SOURCE_DIR})

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(Serial REQUIRED)
find_package(qbrobotics_driver REQUIRED)
find_package(qbshr_ctr REQUIRED)


# Build Settings
add_executable(qbhand_control src/MainHand.cc)


#target_link_libraries(qbhand_control
#        PUBLIC
#        qbshr_ctr::qbshr_ctr
#
#)

ament_target_dependencies(qbhand_control
        std_msgs
        rclcpp
        sensor_msgs
        Serial
        qbrobotics_driver
        qbshr_ctr
)

install(TARGETS
        qbhand_control
        DESTINATION lib/${PROJECT_NAME})


ament_package()
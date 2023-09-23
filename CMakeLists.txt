cmake_minimum_required(VERSION 3.8)
project(ur5_qbhand_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
#add_subdirectory(lib/serial)
#add_subdirectory(lib/qbrobotics-driver)

# Adds excecutable
add_executable(qbhand_control src/
        src/MainHand.cc
)
ament_target_dependencies(qbhand_control rclcpp std_msgs)


install(TARGETS
        qbhand_control
        DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

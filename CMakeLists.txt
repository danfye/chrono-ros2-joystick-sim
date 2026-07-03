cmake_minimum_required(VERSION 3.18)
project(chrono_sim_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Chrono REQUIRED COMPONENTS Vehicle Irrlicht CONFIG)

set(CMAKE_CXX_STANDARD ${CHRONO_CXX_STANDARD})

add_library(chrono_joystick_mapper src/joystick_mapper.cpp)
target_include_directories(chrono_joystick_mapper
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

add_executable(chrono_joy_driver src/simplified_driver.cpp)

target_include_directories(chrono_joy_driver PRIVATE ${CHRONO_INCLUDE_DIRS})
target_compile_options(chrono_joy_driver PRIVATE ${CHRONO_CXX_FLAGS})
target_link_options(chrono_joy_driver PRIVATE ${CHRONO_LINKER_FLAGS})
target_link_libraries(chrono_joy_driver
  chrono_joystick_mapper
  ${CHRONO_LIBRARIES}
)

ament_target_dependencies(chrono_joy_driver
  rclcpp
  sensor_msgs
)

install(TARGETS chrono_joy_driver chrono_joystick_mapper
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  ament_add_gtest(test_joystick_mapper tests/test_joystick_mapper.cpp)
  target_link_libraries(test_joystick_mapper chrono_joystick_mapper)
  target_include_directories(test_joystick_mapper PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

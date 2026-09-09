cmake_minimum_required(VERSION 3.8)
project(stm32_can_ota)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/OtaStatus.msg"
  "srv/StartOta.srv"
  "srv/CancelOta.srv"
  DEPENDENCIES builtin_interfaces
)

add_executable(stm32_ota_node
  src/main.cpp
  src/stm32_ota_node.cpp
  src/ota_client.cpp
  src/can_transport.cpp
  src/ota_protocol.cpp
  src/firmware_image.cpp
  src/firmware_package.cpp
  src/crc16.cpp
  src/crc32.cpp
)

target_compile_features(stm32_ota_node PUBLIC cxx_std_17)

target_include_directories(stm32_ota_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(stm32_ota_node rclcpp)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME}
  "rosidl_typesupport_cpp"
)
target_link_libraries(stm32_ota_node "${cpp_typesupport_target}")

install(TARGETS
  stm32_ota_node
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/download_firmware.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include
)

install(DIRECTORY
  config
  docs
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  find_package(Threads REQUIRED)
  ament_add_gtest(test_ota_host
    test/test_ota_host.cpp
    src/ota_client.cpp
    src/can_transport.cpp
    src/ota_protocol.cpp
    src/firmware_image.cpp
    src/firmware_package.cpp
    src/crc16.cpp
    src/crc32.cpp
  )
  target_include_directories(test_ota_host PRIVATE include)
  target_compile_features(test_ota_host PUBLIC cxx_std_17)
  target_link_libraries(test_ota_host Threads::Threads)
endif()

ament_package()

cmake_minimum_required(VERSION 3.5)
project(usbrelay)


set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "-std=c++11 -g ${CMAKE_CXX_FLAGS}")


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(usbrelay_interfaces REQUIRED)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src)

add_executable(${PROJECT_NAME} src/usbrelay.cpp  src/libusbrelay.c)

target_link_libraries(${PROJECT_NAME} -lhidapi-hidraw)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_11)  # Require C99 and C++11

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  usbrelay_interfaces
)

add_executable(${PROJECT_NAME}_test src/usbrelay_test.cpp)

target_include_directories(${PROJECT_NAME}_test PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

target_compile_features(${PROJECT_NAME}_test PUBLIC c_std_99 cxx_std_11)  # Require C99 and C++11

ament_target_dependencies(${PROJECT_NAME}_test
  rclcpp
  usbrelay_interfaces
)



install(TARGETS ${PROJECT_NAME}  ${PROJECT_NAME}_test
  DESTINATION lib/${PROJECT_NAME})

ament_package()

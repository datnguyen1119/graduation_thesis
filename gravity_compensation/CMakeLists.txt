cmake_minimum_required(VERSION 3.8)
project(gravity_compensation)

# -----------------------------
# Compiler options
# -----------------------------
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# -----------------------------
# Dependencies
# -----------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(can_odrive_interface REQUIRED)
find_package(pinocchio REQUIRED)
find_package(Eigen3 REQUIRED)

# -----------------------------
# Library
# -----------------------------
add_library(${PROJECT_NAME} SHARED
  src/gravity_compensation.cpp
  src/impedance_control.cpp
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# ament metadata dependencies
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  sensor_msgs
  can_odrive_interface
  pinocchio
  Eigen3
)



# Prefer linking modern imported targets if they exist in your environment
# (Pinocchio thường có pinocchio::pinocchio, Eigen3 có Eigen3::Eigen)
# Nếu distro của bạn không cung cấp target này, CMake sẽ báo lỗi -> khi đó bỏ các dòng link này.
target_link_libraries(gravity_compensation
  pinocchio::pinocchio
  Eigen3::Eigen
)



# -----------------------------
# Node executable
# -----------------------------
add_executable(${PROJECT_NAME}_node
  src/gravity_compensation_node.cpp
)

add_executable(simple_gravity_compensation
  src/simple_gravity_compensation_node.cpp
)

target_compile_features(${PROJECT_NAME}_node PUBLIC cxx_std_17)

target_compile_features(simple_gravity_compensation PUBLIC cxx_std_17)

target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
)


ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  sensor_msgs
  can_odrive_interface
)

ament_target_dependencies(simple_gravity_compensation
  rclcpp
  can_odrive_interface
)

# -----------------------------
# Export / Install
# -----------------------------
# Headers
install(
  DIRECTORY include/
  DESTINATION include
)

# Library + Node
install(
  TARGETS
    ${PROJECT_NAME}
    ${PROJECT_NAME}_node
    simple_gravity_compensation
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Export targets so downstream can do:
# find_package(gravity_compensation REQUIRED)
# target_link_libraries(their_target gravity_compensation::gravity_compensation)
install(
  EXPORT export_${PROJECT_NAME}
  DESTINATION share/${PROJECT_NAME}/cmake
  NAMESPACE ${PROJECT_NAME}::
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(rclcpp sensor_msgs can_odrive_interface pinocchio Eigen3)

# -----------------------------
# ament package
# -----------------------------
ament_package()

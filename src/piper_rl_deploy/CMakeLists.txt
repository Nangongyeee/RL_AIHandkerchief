cmake_minimum_required(VERSION 3.8)
project(piper_rl_deploy)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Find PyTorch (LibTorch) - 参考 rl_sar 的写法
find_package(Torch REQUIRED)
message(STATUS "Found PyTorch: ${TORCH_LIBRARIES}")
set(USE_PYTORCH ON)

# Find ONNX Runtime (optional)
find_library(ONNXRUNTIME_LIB onnxruntime HINTS /usr/local/lib /usr/lib)
if(ONNXRUNTIME_LIB)
  message(STATUS "Found ONNX Runtime: ${ONNXRUNTIME_LIB}")
  set(USE_ONNX ON)
else()
  message(WARNING "ONNX Runtime not found. Will build without ONNX support.")
  set(USE_ONNX OFF)
endif()

# Include directories
include_directories(include)

# Create model_loader library (像 rl_sar 一样处理 PyTorch 链接)
add_library(model_loader_lib src/model_loader.cpp)
set_target_properties(model_loader_lib PROPERTIES
    CXX_STANDARD 14
    CXX_STANDARD_REQUIRED ON
)

# Link PyTorch to the library instead of the executable
if(USE_PYTORCH)
  target_link_libraries(model_loader_lib PUBLIC "${TORCH_LIBRARIES}")
  target_compile_definitions(model_loader_lib PRIVATE USE_PYTORCH)
endif()

if(USE_ONNX)
  target_link_libraries(model_loader_lib PUBLIC ${ONNXRUNTIME_LIB})
  target_compile_definitions(model_loader_lib PRIVATE USE_ONNX)
endif()

# Create executable
add_executable(piper_rl_node 
  src/piper_rl_node.cpp
  src/piper_rl_controller.cpp
)

# Set dependencies
ament_target_dependencies(piper_rl_node 
  rclcpp 
  std_msgs 
  geometry_msgs 
  sensor_msgs 
  tf2 
  tf2_ros 
  tf2_geometry_msgs
)

# Link the model_loader library instead of PyTorch directly
target_link_libraries(piper_rl_node model_loader_lib)

# Install targets
install(TARGETS piper_rl_node model_loader_lib
  DESTINATION lib/${PROJECT_NAME}
)

# Install directories
install(DIRECTORY
  config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY
  models/
  DESTINATION share/${PROJECT_NAME}/models
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

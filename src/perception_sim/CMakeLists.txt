cmake_minimum_required(VERSION 3.8)
project(perception_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找编译所需的包
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
# 因为srv文件依赖geometry_msgs，所以这里也需要find_package
find_package(geometry_msgs REQUIRED)

# 定义需要生成的接口文件
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GetObjectPose.srv"
  # 明确指出此接口依赖于其他包中的消息类型
  DEPENDENCIES geometry_msgs
)

# 声明包信息，完成构建
ament_package()
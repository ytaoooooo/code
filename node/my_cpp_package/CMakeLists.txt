cmake_minimum_required(VERSION 3.8)
project(my_cpp_package)

# 这段代码用于检查编译器是否为GNU C++编译器或Clang编译器
# 如果是，则添加编译选项-Wall -Wextra -Wpedantic
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# 可执行文件的名字 
add_executable(cpp_node src/cpp_node.cpp)

# target_link_directories 用于指定可执行文件cpp_node的链接目录
# 这里将cpp_node的链接目录设置为rclcpp的包含目录
# target_link_directories(cpp_node PUBLIC ${rclcpp_INCLUDE_DIRS})

# target_link_libraries 用于指定可执行文件cpp_node所需链接的库
# 这里将cpp_node所需链接的库设置为rclcpp的库
# target_link_libraries(cpp_node ${rclcpp_LIBRARIES})

# ament_target_dependencies 用于指定可执行文件cpp_node的依赖
# 这里将cpp_node的依赖设置为rclcpp
ament_target_dependencies(cpp_node rclcpp)



# install命令用于指定要安装的目标文件
# 这里将可执行文件cpp_node安装到lib/${PROJECT_NAME}目录下
install(TARGETS cpp_node DESTINATION lib/${PROJECT_NAME})



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

ament_package()

# Node
在 ROS 2（Robot Operating System 2） 中，节点（Node） 是系统的基本执行单元，负责处理特定的功能，如传感器数据处理、运动控制或决策逻辑等。ROS 2 采用了分布式架构，多个节点可以在不同的设备上运行，并通过 主题（Topic）、服务（Service）、动作（Action） 和 参数（Parameter） 进行通信。

# ROS2 工作空间构建流程
1. 创建工作空间目录
mkdir -p ros2_ws/src  # 创建工作空间和源码目录
cd ros2_ws/src  # 进入源码目录

2. 创建功能包
ros2 pkg create --build-type ament_python my_package --license Apache-2.0  # 创建Python功能包并指定Apache许可证
# 或
ros2 pkg create --build-type ament_cmake --license Apache-2.0 my_cpp_package  # 创建C++功能包并指定Apache许可证

3. 编写代码
# 在功能包目录下编写节点代码、配置文件等

4. 构建工作空间
cd ~/ros2_ws  # 返回工作空间根目录
colcon build  # 构建所有功能包
# 或
colcon build --packages-select my_package  # 只构建指定功能包
# python构建之后的文件夹及其功能

在成功构建工作空间后，会生成以下几个重要的文件夹：
1. build/
   - 该文件夹包含了构建过程中生成的中间文件和构建产物。它是由构建系统（如colcon）创建的，用于存放编译后的目标文件、库文件等。
2. install/
   - 该文件夹包含了安装后的文件和目录结构。它是构建系统将构建产物安装到此目录中，以便于运行和使用。该目录下会有各个功能包的安装文件，如可执行文件、库文件、配置文件等。
3. log/
   - 该文件夹包含了构建过程中的日志文件。它记录了构建过程中发生的所有事件和输出信息，便于开发者在构建失败时进行调试和排查问题。

这些文件夹在构建过程中自动生成，开发者无需手动创建。通过这些文件夹，开发者可以方便地管理和使用构建产物，并进行调试和优化。
# cpp构建
   自己创建一个build文件夹 cd build  然后 cmake..

5. 设置环境
source install/setup.bash  # 设置环境变量

6. 运行节点
在成功构建工作空间后，可以通过以下步骤运行节点：


ros2 run my_python_package python_node  # 运行节点


注意事项:
- 确保工作空间结构正确
- 正确设置功能包依赖
- 构建前检查代码和配置文件
- 定期清理build和install目录

# ROS2 功能包
功能包（Package）是 ROS 2 中组织代码的基本单元，它包含了节点、库、配置文件等资源。功能包使得代码模块化、可重用，并便于分发和安装。

## 功能包的组成
一个典型的 ROS 2 功能包包含以下内容：
- package.xml: 功能包的配置文件，定义包的基本信息和依赖
- setup.py/CMakeLists.txt: 构建系统配置文件
- 源代码文件: 节点实现、库等
- 启动文件: 用于同时启动多个节点
- 配置文件: 参数配置等
- 测试文件: 单元测试等

## 功能包类型
ROS 2支持两种主要的功能包类型：
1. Python功能包（ament_python）
   - 主要用于Python代码
   - 使用setup.py进行构建配置
   - 适合快速开发和原型设计

2. C++功能包（ament_cmake）
   - 主要用于C++代码
   - 使用CMakeLists.txt进行构建配置
   - 适合性能关键的应用

## 功能包管理命令
- ros2 pkg create: 创建新功能包
- ros2 pkg list: 列出已安装的功能包
- ros2 pkg executables: 查看功能包中的可执行文件
- colcon build: 构建功能包


# source ./devel/setup.bash
   它会更新 PATH 环境变量，使得你可以在终端中直接运行工作空间中的可执行文件。
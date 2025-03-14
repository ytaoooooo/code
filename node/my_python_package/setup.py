# 导入所需的setuptools模块
# setuptools提供了Python包的安装、构建和分发工具
# find_packages用于自动查找所有Python包
# setup是主要的配置函数
from setuptools import find_packages, setup

# 定义功能包名称 - 在整个配置中会多次使用
package_name = 'my_python_package'

# 配置功能包的详细信息
setup(
    # 功能包名称 - 这是安装后的包名,需要与文件夹名一致
    name=package_name,
    
    # 版本号 - 遵循语义化版本规范(major.minor.patch)
    version='0.0.0',
    
    # 自动查找所有Python包
    # exclude=['test']表示排除测试目录
    # 这确保只安装实际的功能代码
    packages=find_packages(exclude=['test']),
    
    # 数据文件配置 - 指定非Python文件的安装位置
    data_files=[
        # 将功能包注册到ROS 2的ament索引系统
        # 这使得ROS 2可以找到并加载这个功能包
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
            
        # 将package.xml复制到安装目录
        # package.xml包含了功能包的元数据
        ('share/' + package_name, ['package.xml']),
    ],
    
    # 声明运行时Python包依赖
    # setuptools是基本依赖,用于安装过程
    install_requires=['setuptools'],
    
    # 允许以zip格式安装包
    # True表示包可以被压缩安装
    zip_safe=True,
    
    # 维护者信息
    maintainer='yangtao',
    maintainer_email='1347781778@qq.com',
    
    # 功能包的简要描述
    # TODO标记表示这里需要添加实际描述
    description='TODO: Package description',
    
    # 开源许可证声明
    # TODO标记表示需要指定具体的许可证
    license='TODO: License declaration',
    
    # 测试相关的依赖
    # pytest是Python的单元测试框架
    tests_require=['pytest'],
    
    # 入口点配置 - 定义可执行文件
    entry_points={
        # console_scripts定义命令行工具
        'console_scripts': [
            # ros2 run pkg_name  python_node 
            # 格式: '命令名 = 包名.模块名:函数名'
            # python_node是一个命令行工具，它会运行my_python_package.ros2_python_node模块中的main函数。
            # 这个函数会初始化ROS2 Python节点，并保持节点活动状态以处理ROS2消息。
            'python_node = my_python_package.ros2_python_node:main',
        ],
    },
)

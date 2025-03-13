from setuptools import find_packages, setup

package_name = 'my_python_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),  # 查找并包含所有不在'test'目录中的包
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),  # 包含资源文件
        ('share/' + package_name, ['package.xml']),  # 包含package.xml文件
    ],
    install_requires=['setuptools'],  # 安装依赖
    zip_safe=True,
    maintainer='yangtao',  # 维护者姓名
    maintainer_email='1347781778@qq.com',  # 维护者邮箱
    description='TODO: Package description',  # 包描述
    license='TODO: License declaration',  # 许可证声明
    tests_require=['pytest'],  # 测试依赖
    entry_points={
        'console_scripts': [
            'python_node = my_python_package.ros2_python_node:main',  # 注册节点启动脚本
        ],
    },
)

from setuptools import find_packages, setup

package_name = 'my_topic_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yt',
    maintainer_email='1347781778@qq.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'novel_pub_node = my_topic_package.novel_pub_node:main',
            'novel_sub_node = my_topic_package.novel_sub_node:main',
        ],
    },
)

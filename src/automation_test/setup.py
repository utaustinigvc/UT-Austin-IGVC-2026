from setuptools import find_packages, setup

package_name = 'automation_test'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='igvc',
    maintainer_email='igvc@todo.todo',
    description='Detect orange in a camera stream and command the robot to move forward.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'orange_forward = automation_test.orange_forward_node:main',
            'lidar_forward = automation_test.lidar_forward_node:main',
        ],
    },
)

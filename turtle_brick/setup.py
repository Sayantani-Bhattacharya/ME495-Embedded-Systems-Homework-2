from setuptools import find_packages, setup

package_name = 'turtle_brick'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml',
                                   'launch/display.launch.xml',
                                   'launch/display.launch.py',
                                   'urdf/unicycle.urdf.xacro',
                                   'config/view_robot.rviz']),
                                   
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sayantani',
    maintainer_email='sayantanibhattacharya2025@u.northwestern.edu',
    description='Turtle brick',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)

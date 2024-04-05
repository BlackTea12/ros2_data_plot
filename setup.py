from setuptools import find_packages, setup

package_name = 'ros2_data_plot'

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
    maintainer='Yaeohn Kim',
    maintainer_email='BlackTea12@github.com',
    description='ros2_data_plot package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'plot_travel_data = ros2_data_plot.plot_travel_data:main',
            'plot_robot_vel = ros2_data_plot.plot_robot_vel:main',
            'recorder = ros2_data_plot.recorder:main',
            'plot_travel_data_precise = ros2_data_plot.plot_travel_data_precise:main',
        ],
    },
)

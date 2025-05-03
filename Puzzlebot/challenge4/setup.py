from setuptools import find_packages, setup

package_name = 'challenge4'

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
    maintainer='daniel',
    maintainer_email='daniel@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "path_generator= challenge4.gen_points:main",
            "Controller= challenge4.close_loop_4_semafor:main",
            "odometry= challenge4.odometry:main",
            "color_detect= challenge4.deteccion_color:main",

        ],
    },
)

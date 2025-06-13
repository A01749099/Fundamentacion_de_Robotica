from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'final_challenge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share',package_name,"launch"), glob("launch/*.[pxy][ymal]*")),
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
        	"controller = final_challenge.controller:main",
        	"pub_webcam = final_challenge.pub_webcam_mux:main",
        	"line_zebra = final_challenge.zebra_otsu:main",
        	"yolo = final_challenge.yolo:main",
        	"semaforo = final_challenge.semaforo:main",
        ],
    },
)

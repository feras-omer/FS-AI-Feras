from setuptools import setup

package_name = 'yolov5_cone_detector'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # YOLOv5 weights live at the repo root
        ('share/' + package_name, ['../best.pt']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Feras Babiker',
    maintainer_email='ferasbabikerjan@gmail.com',
    description='YOLOv5 cone detector for the EUFS simulator ZED camera',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolov5_cone_node = yolov5_cone_detector.yolo_node:main',
        ],
    },
)

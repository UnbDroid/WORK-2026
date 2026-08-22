from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'camera_arm_transform'

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
    maintainer='Lucca',
    maintainer_email='lucca.santos.aguilar@gmail.com',
    description='Nó de transformação geométrica entre câmera e braço',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cube_tf_node = camera_arm_transform.cube_tf_node:main'
        ],
    },
)
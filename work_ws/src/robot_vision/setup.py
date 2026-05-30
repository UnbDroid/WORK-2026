from setuptools import find_packages, setup

package_name = 'robot_vision'

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
    maintainer='lucca',
    maintainer_email='sophia.santos.aguilar@gmail.com',
    description='No de visao para deteccao de cubos com AprilTags e OpenCV',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "vision_node = robot_vision.vision_node:main"
        ],
    },
)

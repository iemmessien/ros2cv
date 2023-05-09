from setuptools import setup

package_name = 'vision_python_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='E',
    maintainer_email='user@todo.todo',
    description='This is a package created to publish and subscribe to image data streamed from a camera device on a topic using OpenCV',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'talker = vision_python_package.vision_publisher:main',
                'listener = vision_python_package.vision_subscriber:main',
                'composed = vision_python_package.vision_composed_nodes:main'
        ],
    },
)

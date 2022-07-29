from setuptools import setup

package_name = 'cv_node'

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
    maintainer='e',
    maintainer_email='e@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'talker = cv_node.cv_publisher:main',
                'listener = cv_node.cv_subscriber:main',
                'composed = cv_node.cv_composed:main'
        ],
    },
)

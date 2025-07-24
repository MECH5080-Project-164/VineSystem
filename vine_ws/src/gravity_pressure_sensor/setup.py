from setuptools import find_packages, setup

package_name = 'gravity_pressure_sensor'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        # Hardware-specific packages are installed at system level in Docker
        # 'smbus', 'spidev', 'RPi.GPIO' - installed via apt in Dockerfile
    ],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pressure_reading_node = gravity_pressure_sensor.pressure_reading_node:main'
        ],
    },
)

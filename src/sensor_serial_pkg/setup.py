from setuptools import find_packages, setup

package_name = 'sensor_serial_pkg'

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
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
		'sensor_node = sensor_serial_pkg.sensor_node:main',
		'processor_node = sensor_serial_pkg.processor_node:main',
		'monitor_node = sensor_serial_pkg.monitor_node:main',
		'database_node = sensor_serial_pkg.database_node:main',
        ],
    },
)

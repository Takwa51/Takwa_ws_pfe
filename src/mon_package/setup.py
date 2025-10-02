from setuptools import find_packages, setup

package_name = 'mon_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/mon_package/launch', ['launch/main_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='samar',
    maintainer_email='samar.rezgui@imt-nord-europe.fr',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
          'mqtt_subscriber_node=mon_package.mqtt_subscriber_node:main',
          'influxdb_logger_node=mon_package.influxdb_logger_node:main',
          'anomaly_detector_node=mon_package.anomaly_detector_node:main',
          'checkpoint_monitor_node=mon_package.checkpoint_monitor_node:main',
          'diagnostic_saver_node=mon_package.diagnostic_saver:main',
          'diagnostic_file_logger_node=mon_package.diagnostic_file_logger:main'
        ],
    },
)

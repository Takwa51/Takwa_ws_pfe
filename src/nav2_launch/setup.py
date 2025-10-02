from setuptools import find_packages, setup

package_name = 'nav2_launch'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ('share/nav2_launch/launch', ['launch/amcl_and_map.launch.py']), 
    ('share/nav2_launch/launch', ['launch/slam.launch.py']), 
    ('share/nav2_launch/launch', ['launch/navigation_launch.py']),
    ('share/nav2_launch/config', ['config/slam_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='samar',
    maintainer_email='samar.rezgui@imt-nord-europe.fr',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [ 'nav2_launch = nav2_launch.nav2_launch:generate_launch_description'
        ],
    },
)

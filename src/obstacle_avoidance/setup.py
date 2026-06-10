from setuptools import find_packages, setup

package_name = 'obstacle_avoidance'

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
    maintainer='vansh',
    maintainer_email='vansht9972@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'obstacle_avoidance_node = obstacle_avoidance.obstacle_avoidance_node:main',
            'status_monitor_node     = obstacle_avoidance.status_monitor_node:main',
            'obstacle_avoid = obstacle_avoidance.obstacle_avoid_node:main',
            'obstacle_tackle = obstacle_avoidance.obstacle_tackle_node:main',
        ],
    },
)

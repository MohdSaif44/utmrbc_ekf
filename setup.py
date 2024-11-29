from setuptools import find_packages, setup

package_name = 'py_odom_pub'

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
    maintainer='saif',
    maintainer_email='mohamedali20@graduate.utm.my',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [

            'imu_data = py_odom_pub.publisher_imu:main',
            'odom_data = py_odom_pub.publisher_odom:main',
            'laser_data = py_odom_pub.publisher_laser:main',
            'local_data = py_odom_pub.publisher_local:main',

        ],
    },
)

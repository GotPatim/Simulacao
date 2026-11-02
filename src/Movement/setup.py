from setuptools import find_packages, setup

package_name = 'Movement'

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
                    'rclpy',
                    'nav_msgs',
                    'geometry_msgs',
                    'transforms3d'
                    ],
    zip_safe=True,
    maintainer='nicolas',
    maintainer_email='nicolaspaleite@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'walk = Movement.walk:main',
#            'walk = Movement.odometria:main',
        ],
    },
)

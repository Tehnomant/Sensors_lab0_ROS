from setuptools import find_packages, setup

package_name = 'fake_encoder_pkg'

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
    maintainer='technomant',
    maintainer_email='technomant@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'encoder = fake_encoder_pkg.fake_encoder:main',
            'driver = fake_encoder_pkg.encoder_driver:main',
            'odometry = fake_encoder_pkg.wheel_odom:main'
        ],
    },
)

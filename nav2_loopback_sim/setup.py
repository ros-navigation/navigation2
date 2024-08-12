from glob import glob
import os

from setuptools import setup


package_name = 'nav2_loopback_sim'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='stevemacenski',
    maintainer_email='steve@opennav.org',
    description='A loopback simulator to replace physics simulation',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'loopback_simulator = nav2_loopback_sim.loopback_simulator:main',
        ],
    },
)

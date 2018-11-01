from setuptools import find_packages
from setuptools import setup

setup(
    name='ros2launch',
    version='0.5.1',
    packages=find_packages(exclude=['test']),
    install_requires=['ros2cli'],
    zip_safe=True,
    author='William Woodall',
    author_email='william@osrfoundation.org',
    maintainer='William Woodall',
    maintainer_email='william@osrfoundation.org',
    url='https://github.com/ros2/launch/tree/master/ros2launch',
    download_url='https://github.com/ros2/launch/releases',
    keywords=[],
    classifiers=[
        'Environment :: Console',
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
    ],
    description='The launch command for ROS 2 command line tools.',
    long_description="""\
The package provides the launch command for the ROS 2 command line tools.""",
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'ros2cli.command': [
            'launch = ros2launch.command.launch:LaunchCommand',
        ],
    }
)

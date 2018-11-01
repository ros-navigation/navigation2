from setuptools import find_packages
from setuptools import setup

setup(
    name='launch_testing',
    version='0.5.1',
    packages=find_packages(exclude=['test']),
    install_requires=['setuptools'],
    zip_safe=True,
    author='Esteve Fernandez',
    author_email='esteve@osrfoundation.org',
    maintainer='Dirk Thomas',
    maintainer_email='dthomas@osrfoundation.org',
    url='https://github.com/ros2/launch',
    download_url='https://github.com/ros2/launch/releases',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Test the output of a process.',
    long_description="""\
This package provides helper scripts for writing tests that use the ROS launch tool.""",
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
)

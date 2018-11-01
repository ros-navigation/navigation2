from setuptools import find_packages
from setuptools import setup

setup(
    name='launch_ros',
    version='0.5.1',
    packages=find_packages(exclude=['test']),
    install_requires=[
        'setuptools',
        'ament_index_python',
        'launch',
        'osrf_pycommon',
        'pyyaml',
    ],
    zip_safe=True,
    author='William Woodall',
    author_email='william@osrfoundation.org',
    maintainer='William Woodall',
    maintainer_email='william@osrfoundation.org',
    url='https://github.com/ros2/launch',
    download_url='https://github.com/ros2/launch/releases',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='ROS specific extensions to `launch`.',
    long_description=(
        'This package provides ROS specific extensions to the launch package.'
    ),
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
)

from setuptools import setup

package_name = 'turtlebot3_rl'

setup(
    name=package_name,
    version='0.0.1',
    packages=[],
    py_modules=[
        'random_crawl_train',
        'random_crawl',
        'turtlebot3_env',
        'dqn',
        'parameters',
         ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Mohammad Haghighipanah',
    author_email='mohammad.haghighipanah@intel.com',
    maintainer='Mohammad Haghighipanah',
    maintainer_email='mohammad.haghighipanah@intel.com',
    keywords=['ROS'],
    classifiers=[
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Examples running Turtlebot3 in gazebo with RL',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'random_crawl_train = random_crawl_train:main',
            'random_crawl = random_crawl:main',
	    'turtlebot3_env = turtlebot3_env',

        ],
    },
)

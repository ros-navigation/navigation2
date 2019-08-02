from setuptools import setup

package_name = 'nav2_turtlebot3_rl'

setup(
    name=package_name,
    version='0.0.1',
    packages=[],
    py_modules=[
        'random_crawl_train',
        'random_crawl',
        'random_crawl_action',
        'turtlebot3_env',
        'dqn',
        'parameters',
         ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
	    ('share/' + package_name + '/saved_models',\
        ['saved_models/random_crawl_burger.h5','saved_models/random_crawl_waffle.h5']),
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
            'random_crawl_action = random_crawl_action:main',
        ],
    },
)

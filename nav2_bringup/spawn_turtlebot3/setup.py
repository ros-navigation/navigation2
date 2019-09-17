from setuptools import setup

PACKAGE_NAME = 'spawn_turtlebot3'

setup(
    name=PACKAGE_NAME,
    version='1.0.0',
    package_dir={'': 'src'},
    packages=[PACKAGE_NAME],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + PACKAGE_NAME]),
        ('share/' + PACKAGE_NAME, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'spawn_turtlebot = spawn_turtlebot3.spawn_turtlebot:main',
        ],
    },
)

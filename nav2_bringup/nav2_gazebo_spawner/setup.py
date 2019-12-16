from setuptools import setup

PACKAGE_NAME = 'nav2_gazebo_spawner'

setup(
    name=PACKAGE_NAME,
    version='0.3.0',
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
            'nav2_gazebo_spawner = nav2_gazebo_spawner.nav2_gazebo_spawner:main',
        ],
    },
)

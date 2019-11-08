from setuptools import setup

PACKAGE_NAME = 'spawn_robot'

setup(
    name=PACKAGE_NAME,
    version='0.2.4',
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
            'spawn_robot = spawn_robot.spawn_robot:main',
        ],
    },
)

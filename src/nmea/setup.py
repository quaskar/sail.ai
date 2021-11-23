from setuptools import setup

package_name = 'nmea'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Sebastian Pliefke',
    maintainer_email='sebi@5in1boot.de',
    description='Ros toolkit providing nmea data stream related nodes',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = nmea.publisher_member_function:main',
            'nmea_file = nmea.nmea_file:main',
            'nmea_udp = nmea.nmea_udp:main',
            'nmea_decoder = nmea.nmea_decoder:main',
            'nmea_udp_sender = nmea.nmea_udp_sender:main',
            'listener = nmea.subscriber_member_function:main',
        ],
    },
)

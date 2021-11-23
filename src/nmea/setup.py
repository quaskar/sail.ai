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
            'nmea_file_receiver = nmea.nmea_file_receiver:main',
            'nmea_udp_receiver = nmea.nmea_udp_receiver:main',
            'nmea_decoder = nmea.nmea_decoder:main',
            'nmea_udp_sender = nmea.nmea_udp_sender:main',
        ],
    },
)

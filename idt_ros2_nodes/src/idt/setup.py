from setuptools import setup

package_name = 'idt'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kjeld Jensen',
    maintainer_email='kjen@sdu.dk',
    description='Introduction to Drone Technology (IDT), SDU UAS Center, University of Southern Denmark',
    license='BSD 3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                 'hello_drone = idt.hello_drone:main',
                 'get_global_pos = idt.get_global_pos:main',
                 'get_radio_rssi = idt.get_radio_rssi:main',
                 'rssi_logger = idt.rssi_logger:main',
        ],
    },
)

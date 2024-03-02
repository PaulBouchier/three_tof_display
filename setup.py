from setuptools import find_packages, setup

package_name = 'three_tof_display'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Paul Bouchier',
    maintainer_email='paul.bouchier@gmail.com',
    description='Display the output of three ToF 8x8 range sensors',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'three_tof_display = three_tof_display.three_tof_display:main',
            'range_pub = three_tof_display.range_pub:main'
        ],
    },
)

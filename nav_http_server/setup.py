from setuptools import setup
from glob import glob
import os

package_name = 'nav_http_server'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='huajuan',
    maintainer_email='zhj630985214@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "nav_http_server = nav_http_server.nav_http_server:main",
            "loc_http_server = nav_http_server.loc_http_server:main"
        ],
    },
)

from setuptools import setup
import os
from glob import glob

package_name = 'macadamia_nav'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/resource', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your@email.com',
    description='Macadamia navigation project for ROSBot/Panther using SLAM and Nav2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'row_follower = macadamia_nav.row_follower:main',
            'macadamia_navigator = macadamia_nav.macadamia_navigator:main',
        ],
    },
)

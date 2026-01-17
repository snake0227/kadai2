from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'kadai2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='snake',
    maintainer_email='github0227@gamil.com',
    description='package for practice',
    license='BSD-3-Clause',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'system_monitor = kadai2.monitor:main',
            'system_display = kadai2.display_node:main',
        ],
    },
)

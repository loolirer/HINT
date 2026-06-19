import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'depth_anything'


def package_data_files(directory):
    entries = []
    for path in glob(os.path.join(directory, '**', '*'), recursive=True):
        if os.path.isfile(path):
            install_dir = os.path.join('share', package_name, os.path.dirname(path))
            entries.append((install_dir, [path]))
    return entries


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ] + package_data_files('models'),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "depth_anything = depth_anything.depth_anything:main",
        ],
    },
)

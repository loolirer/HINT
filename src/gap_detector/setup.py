from setuptools import find_packages, setup

package_name = 'gap_detector'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='loolirer',
    maintainer_email='lorenzo.oliveira@ee.ufcg.edu.br',
    description='Depth-image gap detector for safe navigation direction',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'gap_detector = gap_detector.gap_detector:main',
        ],
    },
)

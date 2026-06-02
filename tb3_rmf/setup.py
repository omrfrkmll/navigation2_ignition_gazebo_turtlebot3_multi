import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'tb3_rmf'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(where='.', include=['tb3_rmf', 'tb3_rmf.*']),
    package_dir={'': '.'},
    data_files=[
        (f'share/{package_name}', ['package.xml']),
        (f'share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}/launch', glob('launch/*.launch.py')),
        (f'share/{package_name}/config', glob('config/*.yaml')),
        (os.path.join('share', package_name, 'map'), glob('map/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Arshad Mehmood',
    maintainer_email='arshadm78@yahoo.com',
    description='RMF integration and fleet adapters for TurtleBot3 multi-robot simulation',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'nav2_fleet_adapter = tb3_rmf.nav2_fleet_adapter:main',
            'dispatch_task = tb3_rmf.dispatch_task:main',
            'monitor_tasks = tb3_rmf.monitor_tasks:main',
            'rmf_marker_transformer = tb3_rmf.rmf_marker_transformer:main',
        ],
    },
)

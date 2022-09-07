from setuptools import setup
import os
from glob import glob

package_name = 'hunav_evaluator'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Noé Pérez-Higueras',
    maintainer_email='noeperez@upo.es',
    description='This package collects the data of the hunav simulations and computes different metrics',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hunav_evaluator_node = hunav_evaluator.hunav_evaluator_node:main'
        ],
    },
)

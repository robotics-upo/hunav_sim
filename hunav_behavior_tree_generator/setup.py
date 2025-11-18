from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'hunav_behavior_tree_generator'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/prompts', glob(f'{package_name}/prompts/*.txt'))
    ],
    install_requires=[
        'setuptools',
        'openai',
        'rclpy',
        'PyYAML',
    ],
    zip_safe=True,
    maintainer='Quentin Dury',
    maintainer_email='quentin.dury@cpe.fr',
    description='LLM-based behavior tree and scenario generator for HuNav simulation',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'generator_cli = hunav_behavior_tree_generator.generator_cli:main',
        ],
    },
)
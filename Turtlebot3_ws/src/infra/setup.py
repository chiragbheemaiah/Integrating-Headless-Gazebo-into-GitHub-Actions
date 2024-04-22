from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'infra'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vscode',
    maintainer_email='chirag.bheemaiah@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publish_initial_pose = infra.publish_initial_pose:main',
            'publish_goal_success = infra.publish_goal_success:main',
            'publish_goal_failure = infra.publish_goal_failure:main',
        ],
    },
)

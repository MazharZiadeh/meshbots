import os
from glob import glob
from setuptools import setup

package_name = 'meshbots'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
        (os.path.join('share', package_name, 'models'), glob('models/*.sdf.template')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'missions'), glob('missions/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mazhar',
    maintainer_email='mziadh99@gmail.com',
    description='Decentralized mesh multi-robot delivery team (ROS 2 + Gazebo)',
    license='MIT',
    entry_points={
        'console_scripts': [
            'radio_channel = meshbots.radio_channel:main',
            'mesh_radio = meshbots.mesh_radio:main',
            'localizer = meshbots.localizer:main',
            'eval_metrics = meshbots.eval_metrics:main',
            'batch_metrics = meshbots.batch_metrics:main',
            'mapper = meshbots.mapper:main',
            'swarm = meshbots.swarm:main',
            'navigator = meshbots.navigator:main',
        ],
    },
)

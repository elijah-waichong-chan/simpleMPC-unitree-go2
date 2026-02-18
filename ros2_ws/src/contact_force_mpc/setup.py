from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'contact_force_mpc'
model_files = [
    os.path.relpath(path, package_name)
    for path in glob(os.path.join(package_name, 'models', '**', '*'), recursive=True)
    if os.path.isfile(path)
]

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    package_data={
        package_name: model_files,
    },
    include_package_data=True,
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='elijah',
    maintainer_email='chanwaichong0352@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'contact_force_mpc = contact_force_mpc.locomotion_node:main',
            'mpc_node = contact_force_mpc.mpc_node:main',
            'qdq_plotter = contact_force_mpc.qdq_plotter:main',
        ],
    },
)

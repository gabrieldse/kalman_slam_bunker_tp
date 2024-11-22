from setuptools import setup
import subprocess, os, platform
from glob import glob


package_name = 'pol_bunker'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        (os.path.join('share',package_name,'launch'),glob('launch/*.py')),
        (os.path.join('share',package_name,'launch'),glob('launch/*.xml')),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name,'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name,'meshes'), glob('meshes/*.stl')),
        (os.path.join('share', package_name,'meshes'), glob('meshes/*.dae')),
        (os.path.join('share', package_name,'params'), glob('params/*.yaml')),
        (os.path.join('share', package_name,'models/cohoma1'), glob('models/cohoma1/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "node_bno055=pol_bunker.node_bno055:main",
            "gotogps_bunker=pol_bunker.gotogps_bunker:main",
            "my_navsat_transform=pol_bunker.my_navsat_transform:main",
            "apriltag_detect=pol_bunker.apriltag_detect:main",
            "kalman1=pol_bunker.kalman1:main"
        ],
    },
)

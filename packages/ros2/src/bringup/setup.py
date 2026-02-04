import os.path as osp
from glob import glob
from setuptools import find_packages, setup

package_name = 'bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (osp.join("share", package_name, "launch"), glob("launch/*")),
        (osp.join("share", package_name, "config"), glob("config/*")),
        (osp.join("share", package_name, "hook"), ["hooks/dotenv.sh"]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='act',
    maintainer_email='drobinson4105@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)

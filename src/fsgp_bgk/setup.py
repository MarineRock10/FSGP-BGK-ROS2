from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'fsgp_bgk'

setup(
    name=package_name,
    version='0.1.0',
    # The Python source lives under python/ rather than a top-level
    # package directory, so we remap the root.
    package_dir={'': 'python'},
    packages=find_packages(where='python'),
    py_modules=[
        'f_sgp_bgk',
        'node_ros1',
        'node_ros2',
        'traversability_service',
    ],
    data_files=[
        # ament index marker so ros2 pkg list can find us
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # install the YAML config next to the share directory
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=[
        'setuptools',
        'numpy',
        'scikit-learn',
        'torch',
        'gpytorch',
        'pyyaml',
    ],
    zip_safe=True,
    maintainer='Senming Tan',
    maintainer_email='senmingtan5@gmail.com',
    description='FSGP-BGK traversability analysis (Python algorithm + ROS 2 service)',
    license='MIT',
    entry_points={
        'console_scripts': [
            'traversability_service = traversability_service:main',
        ],
    },
)

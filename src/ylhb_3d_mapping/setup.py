from glob import glob
from setuptools import setup

package_name = 'ylhb_3d_mapping'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/docs', glob('docs/*.md')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='liaojingwu20041031',
    maintainer_email='206929594+liaojingwu20041031@users.noreply.github.com',
    description='ZED SDK spatial mapping export node.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'zed_spatial_mapping_node = ylhb_3d_mapping.zed_spatial_mapping_node:main',
        ],
    },
)

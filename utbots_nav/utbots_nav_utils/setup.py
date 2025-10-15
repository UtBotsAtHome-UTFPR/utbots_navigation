from setuptools import setup,find_packages
import os
from glob import glob
package_name = 'utbots_nav_utils'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ehg2004',
    maintainer_email='enzoholzmanngaio@alunos.utfpr.edu.br',
    description='TODO: Package description',
    license='Apache-2.0',
    # tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'save_waypoint_service = utbots_nav_utils.save_waypoint_service:main',
            'nav_to_wp = utbots_nav_utils.nav_to_wp:main'
        ],
    },
    extras_require={
    'test': ['pytest', 'other-test-deps'],
    },

)

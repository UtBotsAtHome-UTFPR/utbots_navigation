from setuptools import find_packages, setup

package_name = 'bridge_navigate_to_pose'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='laser',
    maintainer_email='laser@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
	   'bridge_nav = bridge_navigate_to_pose.bridge_nav:main',
       'nav_bridge_test = bridge_navigate_to_pose.nav_bridge_test:main',
        ],
    },
)

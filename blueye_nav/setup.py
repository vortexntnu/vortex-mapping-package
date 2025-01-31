from setuptools import find_packages, setup
import os

package_name = 'blueye_nav'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        (os.path.join('share', package_name, 'launch'), ['launch/nav_launch.py']),
        (os.path.join('share', package_name, 'config'), ['config/global_costmap.yaml', 'config/local_costmap.yaml', 'config/my_nav2_params.yaml', 'config/planner.yaml', 'config/controller.yaml', 'config/recovery.yaml']),
    
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sam',
    maintainer_email='sam@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)

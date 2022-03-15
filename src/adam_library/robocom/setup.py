from setuptools import setup
import os
from glob import glob

package_name = 'robocom'



setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['record_configuration.xml']),
        (os.path.join('share', package_name), glob('launch/*_launch.py'))

        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ed',
    maintainer_email='ed@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'externalreceiver = robocom.external_command_receiver:main',
            'commandlistmanager = robocom.command_list_manager:main',
            'commandexecutionmanager = robocom.command_execution_manager_2:main',
            'gripperexecutionnode = robocom.gripper_execution_node:main',
            'managertogripper= robocom.manager_to_gripper_service:main',
            'rtdestatepublisher= robocom.rtde_state_publisher:main',
            'robotexecutionnode= robocom.robot_execution_node:main',
            'robotlooper= robocom.robot_looper:main',
            'queueshouter= robocom.queueshouter:main',
            
            
        ],
    },
)

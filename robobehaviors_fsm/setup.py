from setuptools import find_packages, setup

package_name = 'robobehaviors_fsm'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/fsm_bringup.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jliu',
    maintainer_email='jliu3@olin.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop = robobehaviors_fsm.teleop:main',
            'rviz_tutorial = robobehaviors_fsm.rviz_tutorial:main',
            'drive_square = robobehaviors_fsm.drive_square:main',
            'wall_follower = robobehaviors_fsm.wall_follower:main',
            'finite_state_controller = robobehaviors_fsm.finite_state_controller:main',
            'keyboard_estop = robobehaviors_fsm.keyboard_estop:main',
            'bumper_estop = robobehaviors_fsm.bumper_estop:main',
            'person_following = robobehaviors_fsm.person_following:main',
        ],
    },
)

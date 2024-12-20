from setuptools import find_packages, setup

package_name = 'lab_1_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
   data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/urdf', ['urdf/robot.urdf']),
        ('share/' + package_name + '/launch', ['launch/display.launch.py']),
        ('share/' + package_name + '/launch', ['launch/gazebo_sim.launch.py']),
        ('share/' + package_name + '/rviz', ['rviz/urdf.rviz']),
        ('share/' + package_name + '/worlds', ['worlds/my_world.sdf']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='surafel',
    maintainer_email='surafel.s.tadesse@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'move_robot = lab_1_package.move_robot:main',
            'detect_objects = lab_1_package.detect_objects:main',
        ],
    },
)
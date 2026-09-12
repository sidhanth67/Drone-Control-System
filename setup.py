import os
from glob import glob
from setuptools import setup


package_name = 'drone_control_pkg'

# Helper function to recursively include nested folders (like materials/textures)
def get_data_files(source_dir, target_dir):
    data_files = []
    if os.path.exists(source_dir):
        for root, dirs, files in os.walk(source_dir):
            if files:
                install_dir = os.path.join(target_dir, os.path.relpath(root, source_dir))
                file_list = [os.path.join(root, f) for f in files]
                data_files.append((install_dir, file_list))
    return data_files

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
data_files=[
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    # Include launch files
    (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    # Include world files
    (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
    # Recursively include all model files, textures, and scripts safely
    *get_data_files('models', os.path.join('share', package_name, 'models')),
],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Mini drone PID control package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'controller_node = drone_control_pkg.controller_node:main',
            'aruco_tracker_node = drone_control_pkg.aruco_tracker_node:main',
        ],
    },
)
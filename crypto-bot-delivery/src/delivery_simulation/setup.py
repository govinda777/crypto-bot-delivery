from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'delivery_simulation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]), # Standard resource registration
        ('share/' + package_name, ['package.xml']),
        # Include all launch files from the launch directory
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.xml'))),
        # Include all world files from the worlds directory
        (os.path.join('share', package_name, 'worlds'), glob(os.path.join('worlds', '*.world'))),
        (os.path.join('share', package_name, 'worlds'), glob(os.path.join('worlds', '*.sdf'))),
        # Include all model files from the models directory (URDF, SDF, meshes, etc.)
        # This needs to be recursive for models with subdirectories (e.g., meshes, textures)
        # For simplicity here, we'll glob common model file types directly in models/
        # A more robust solution might involve walking the directory.
        (os.path.join('share', package_name, 'models'), glob(os.path.join('models', '*.urdf*'))),
        (os.path.join('share', package_name, 'models'), glob(os.path.join('models', '*.sdf'))),
        (os.path.join('share', package_name, 'models'), glob(os.path.join('models', '*.xacro'))),
        (os.path.join('share', package_name, 'models'), glob(os.path.join('models', '*.dae'))), # Collada meshes
        (os.path.join('share', package_name, 'models'), glob(os.path.join('models', '*.stl'))), # STL meshes
        # If models have subdirectories like 'meshes' or 'materials':
        # (os.path.join('share', package_name, 'models', 'meshes'), glob(os.path.join('models', 'meshes', '*'))),
        # (os.path.join('share', package_name, 'models', 'textures'), glob(os.path.join('models', 'textures', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='todo.todo@example.com',
    description='ROS2 package for crypto-bot-delivery Gazebo simulation',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Example: 'my_sim_node = delivery_simulation.my_sim_node:main',
        ],
    },
)

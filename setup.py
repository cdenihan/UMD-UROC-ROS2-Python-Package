from setuptools import find_packages, setup

package_name = 'py_uroc'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/visualize.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cdenihan',
    maintainer_email='cdenihan@proton.me',
    description='UMD UROC ROS2 Python Package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'foxglove_3d_path_visualization = py_uroc.foxglove_3d_path_visualization:main'
        ],
    },
)

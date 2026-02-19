from setuptools import find_packages, setup

package_name = 'arpa_bt_executor'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Zack Allen',
    maintainer_email='zackallen@example.com',
    description='Behavior tree executor for constrained screw placement sequence.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'bt_executor_node = arpa_bt_executor.bt_executor_node:main',
        ],
    },
)

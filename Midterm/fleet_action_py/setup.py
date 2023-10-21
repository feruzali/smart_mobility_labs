from setuptools import find_packages, setup

package_name = 'fleet_action_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Feruz',
    maintainer_email='feruz.privet@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "fleet_management_server_cli = fleet_action_py.fleet_management_server_cli:main",
            "fleet_management_client_cli = fleet_action_py.fleet_management_client_cli:main",        
        ],
    },
)

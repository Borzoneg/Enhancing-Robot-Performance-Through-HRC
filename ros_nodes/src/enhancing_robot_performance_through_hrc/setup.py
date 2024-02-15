from setuptools import find_packages, setup

package_name = 'enhancing_robot_performance_through_hrc'

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
    maintainer='fluently',
    maintainer_email='borzoneg@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gui = enhancing_robot_performance_through_hrc.gui:main',
            'tree = enhancing_robot_performance_through_hrc.behaviour_tree:main',
            'srvs = enhancing_robot_performance_through_hrc.services_placeholder:main',
        ],
    },
)

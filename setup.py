from setuptools import setup

package_name = 'c82_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'launch/simulation.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='logan',
    maintainer_email='naidoo.logan@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)

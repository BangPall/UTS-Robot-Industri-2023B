from setuptools import find_packages, setup

package_name = 'kontrol_waypoint'

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
    maintainer='bangpall',
    maintainer_email='rianpalma01@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'manual_control = kontrol_waypoint.manual_control:main',
            'mover_with_track = kontrol_waypoint.mover_with_track:main',
        ],
    },
)

from setuptools import setup

package_name = 'rumble'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nhewitt',
    maintainer_email='hewittna@oregonstate.edu',
    description='TODO: Package description',
    license='GNU Public License 3.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rumble_node = rumble.rumble_node:main',
        ],
    },
)

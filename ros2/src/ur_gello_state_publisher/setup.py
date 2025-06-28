from setuptools import find_packages, setup
import glob

package_name = 'ur_gello_state_publisher'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob.glob("config/*.yaml")),
        ("share/" + package_name + "/launch", ["launch/main.launch.py"]),
    ],
    install_requires=['setuptools', 'dynamixel_sdk'],
    zip_safe=True,
    maintainer='Jemuel Stanley Premkumar',
    maintainer_email='jemuelstanley47@gmail.com',
    description='Publishes the state of the GELLO scaled UR teleoperation arm.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        "console_scripts": [
            "gello_publisher = ur_gello_state_publisher.gello_publisher:main"
        ],
    },
)

from setuptools import find_packages, setup

package_name = 'prophesee_evk4_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml'])
    ],
    install_requires=['setuptools', 'rosidl_default_generators'],
    zip_safe=True,
    maintainer='Ricard',
    maintainer_email='ricard.marsal@uni.lu',
    description='package for getting events of evk4 and publishing them in the ros2 network',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "evk4_publisher = prophesee_evk4_driver.evk4_publisher:main",
            "evk4_subscriber = prophesee_evk4_driver.evk4_subscriber:main"
        ],
    },
)

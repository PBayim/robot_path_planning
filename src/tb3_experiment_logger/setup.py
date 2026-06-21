from setuptools import find_packages, setup

package_name = 'tb3_experiment_logger'

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
    maintainer='pat-ubuntu',
    maintainer_email='patrickwbayim@gmail.com',
    description='Passive CSV metrics logger for NavigateToPose runs',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'metrics_logger = tb3_experiment_logger.metrics_logger:main',
        ],
    },
)

from setuptools import find_packages, setup

package_name = 'sensors_controller'

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
    maintainer='ksawery',
    maintainer_email='85024534+KsaweryAiR@users.noreply.github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tf_marker_publisher = sensors_controller.tf_marker_publisher:main',
            'tf_text_publisher = sensors_controller.tf_text_publisher:main',
        ],
    },
)

from setuptools import setup

package_name = 'test_nodes'

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
    maintainer='juyong',
    maintainer_email='juyong3393@snu.ac.kr',
    description='test nodes',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'test_01 = test_nodes.test_01_takeoff_and_land:main',
        ],
    },
)

from setuptools import find_packages, setup

package_name = 'dvrk_mtm_gc'

setup(
    name=package_name,
    version='1.1.0',
    packages=find_packages(exclude=['test']) + ['scripts'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', [
            'config/data_collection.json',
            'config/gc_controller.json',
            'config/gc_test.json',
            'config/mlse.json',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Anton Deguet',
    maintainer_email='adeguet1@jhu.edu',
    description='Gravity compensation for dVRK MTM arms',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'collect_data = scripts.collect_data:main',
            'identify_params = scripts.identify_params:main',
            'define_workspace = scripts.define_workspace:main',
            'test = scripts.test:main',
        ],
    },
)

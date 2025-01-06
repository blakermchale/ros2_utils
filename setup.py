from setuptools import setup

package_name = 'ros2_utils'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'jinja2'],
    zip_safe=True,
    maintainer='bmchale',
    maintainer_email='mchale.blake@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            f'relay = {package_name}.relay:main',
        ],
    },
)

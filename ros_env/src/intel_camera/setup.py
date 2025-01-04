from setuptools import find_packages, setup

package_name = 'intel_camera'

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
    maintainer='uli',
    maintainer_email='A00837223@tec.mx',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "intel_pub = intel_camera.intel_pub:main",
            "intel_sub = intel_camera.intel_sub:main"
        ],
    },
)

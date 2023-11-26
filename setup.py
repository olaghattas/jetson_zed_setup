from setuptools import find_packages, setup

package_name = 'jetson_zed_setup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='olagh',
    maintainer_email='olaghattas@hotmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'run_pf = jetson_zed_setup.run_pf:main',
            'run_without_pf = jetson_zed_setup.run_without_pf:main',
        ],
    },
)

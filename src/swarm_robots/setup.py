from setuptools import setup

package_name = 'swarm_robots'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    py_modules=[],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Swarm robotics custom package',
    license='License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = swarm_robots.talker:main',
            'listener = swarm_robots.listener:main',
        ],
    },
)

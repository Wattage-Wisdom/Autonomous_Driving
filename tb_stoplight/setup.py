from setuptools import setup

package_name = 'tb_stoplight'

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
    maintainer='william',
    maintainer_email='william@todo.todo',
    description='Turtlebot3 implementation of a standard traffic stoplight behaviour.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_publisher = tb_stoplight.image_publisher:main',
            'image_subscriber = tb_stoplight.image_subscriber:main',
            'teleop = tb_stoplight.teleop:main',
            'interpretor = tb_stoplight.stoplight_interpretor:main',
            'override = tb_stoplight.manual_override:main',
            'ml_interpretor = tb_stoplight.ml_interpretor:main'
        ],
    },
)

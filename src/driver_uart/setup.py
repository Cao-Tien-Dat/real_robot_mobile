from setuptools import setup

package_name = 'driver_uart'

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
    maintainer='jetson',
    maintainer_email='jetson@todo.todo',
    description='Driver UART node for diffdrive robot',
    license='MIT',
    tests_require=['pytest'],
    
    entry_points={
        'console_scripts': [
            'driver_uart = driver_uart.driver_uart_node:main',
        ],
    },

)

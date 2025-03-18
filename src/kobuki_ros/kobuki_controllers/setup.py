from setuptools import find_packages, setup
import os

package_name = 'kobuki_controllers'

def get_data_matlab(source_dir, install_dir):
    data_files = []
    for root, dirs, files in os.walk(source_dir):
        for file in files:
            file_path = os.path.join(root, file)
            install_path = os.path.join(install_dir, os.path.relpath(root, source_dir))
            data_files.append((install_path, [file_path]))
    return data_files

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        *get_data_matlab('controllers_matlab', os.path.join('share', package_name, 'controllers_matlab')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nilton',
    maintainer_email='nilton@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "LQR = kobuki_controllers.LQR:main",
            "LMIs = kobuki_controllers.LMIs:main",
            "LMIsRes = kobuki_controllers.LMIsRes:main",
            "CustoGar = kobuki_controllers.CustoGar:main"
        ],
    },
)

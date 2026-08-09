from setuptools import find_packages, setup

package_name = 'line_orchestrator'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Santiago Machado',
    maintainer_email='santiagomach2@hotmail.com',
    description='Orquestador de la linea y nodo de prueba de estacion.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'orchestrator = line_orchestrator.orchestrator:main',
            'station_mock = line_orchestrator.station_mock:main',
        ],
    },
)

from setuptools import find_packages, setup

paquete = 'station1_control'

setup(
    name=paquete,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + paquete]),
        ('share/' + paquete, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Santiago Machado',
    maintainer_email='santiagomach2@hotmail.com',
    description='Cinematica propia y control punto a punto de la Estacion 1.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'punto_a_punto = station1_control.punto_a_punto:main',
            'comparar_moveit = station1_control.comparar_moveit:main',
            'nodo_estacion = station1_control.nodo_estacion:main',
            'prueba_cinematica = station1_control.prueba_cinematica:main',
        ],
    },
)

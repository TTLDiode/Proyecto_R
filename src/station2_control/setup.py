from setuptools import find_packages, setup

paquete = 'station2_control'

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
    maintainer='Diego Oxman Sabogal',
    maintainer_email='paolfoni@gmail.com',
    description='Cinematica propia, control punto a punto, vision y nodo de '
                'la Estacion 2.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'punto_a_punto = station2_control.punto_a_punto:main',
            'nodo_estacion = station2_control.nodo_estacion:main',
            'vision = station2_control.vision:main',
            'prueba_estacion = station2_control.prueba_estacion:main',
            'prueba_cinematica = station2_control.prueba_cinematica:main',
            'comparar_moveit = station2_control.comparar_moveit:main',
            'alimentador_linea = station2_control.alimentador_linea:main',
        ],
    },
)

from setuptools import find_packages, setup

package_name = 'station3_control'

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
    maintainer='judavca',
    maintainer_email='juandavidguerracabrera@gmail.com',
    description='Cinematica propia, movimiento punto a punto y nodo de contrato de la Estacion 3 (RRP).',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'punto_a_punto = station3_control.punto_a_punto:main',
            'nodo_estacion = station3_control.nodo_estacion:main',
        ],
    },
)

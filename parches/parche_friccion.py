import pathlib, sys
f = pathlib.Path(sys.argv[1]); t = f.read_text()
viejo = """    <limit lower="0.0" upper="${carrera_d3}"
           velocity="${vel_pris}" effort="${esfuerzo_pris}"/>
    <dynamics damping="2.0" friction="0.5"/>"""
nuevo = """    <limit lower="0.0" upper="${carrera_d3}"
           velocity="${vel_pris}" effort="${esfuerzo_pris}"/>
    <!-- La friccion no es un numero de relleno: es lo que hace del eje un eje
         autoblocante. El husillo trabaja con un rendimiento directo de 0.2, y
         un husillo por debajo de 0.5 no se puede retroaccionar: el carro no
         baja solo aunque se corte la corriente. Modelarlo con 0.5 N, por
         debajo de los 2.17 N que pesa el conjunto, describia justo lo
         contrario, un eje que se descuelga, y en el simulador se veia: al
         descender, el carro adelantaba 9 mm a su consigna porque caia mas
         rapido de lo que el actuador lo bajaba. Con 4.0 N, por encima del
         peso, el eje se sostiene solo y el actuador conserva 61 N para
         moverlo. -->
    <dynamics damping="2.0" friction="4.0"/>"""
assert t.count(viejo) == 1, 'no encuentro la dinamica de joint_3'
f.write_text(t.replace(viejo, nuevo)); print('friccion de d3 ajustada')

"""Cuelga tool0 del carro y no de la mordaza.

Motivo. tool0 es, por convencion de ROS-Industrial, un sistema fijo solidario
al ultimo eslabon del brazo. En la Entrega 1 quedo colgando de gripper_link,
que gira con el servo, y eso tiene dos consecuencias indeseables:

  1. El punto de trabajo se mueve al abrir la mordaza. Con el servo en su tope
     de 0.5 rad, tool0 se desplaza 35*sin(0.5) = 16.8 mm, que es casi el
     presupuesto entero de precision de la linea.
  2. El grupo de planificacion deja de tener tres grados de libertad. MoveIt
     recorre la cadena desde la base hasta el extremo y se encuentra cuatro
     articulaciones, de modo que el servo entra en la planificacion. La guia
     pide tres grados de libertad y el servo no es uno de ellos.

Con la mordaza cerrada, que es como se recorrio el arbol TF en la Entrega 1,
la pose de tool0 es exactamente la misma que antes. El cambio no mueve nada:
solo deja de mover lo que no deberia moverse.
"""
import sys, pathlib

f = pathlib.Path(sys.argv[1]); t = f.read_text()

viejo = """  <!-- Punto de referencia del efector, donde se sujeta la pieza. -->
  <joint name="${p}tool0_joint" type="fixed">
    <parent link="${p}gripper_link"/>
    <child  link="${p}tool0"/>
    <origin xyz="0 0 ${-gripper_z}"/>
  </joint>
  <link name="${p}tool0"/>
"""
nuevo = """  <!-- Punto de referencia del efector, donde se sujeta la pieza.
       Cuelga del carro y no de la mordaza: tool0 es un sistema fijo del
       brazo, y si girase con el servo el punto de trabajo se desplazaria
       16.8 mm al abrir la pinza y el grupo de planificacion pasaria a tener
       cuatro articulaciones en lugar de los tres grados de libertad que pide
       la guia. Con la mordaza cerrada la pose es identica a la de la
       Entrega 1. -->
  <joint name="${p}tool0_joint" type="fixed">
    <parent link="${p}link_3"/>
    <child  link="${p}tool0"/>
    <origin xyz="0 0 ${-(vastago_largo + gripper_z)}"/>
  </joint>
  <link name="${p}tool0"/>
"""
assert t.count(viejo) == 1, 'no encuentro la definicion de tool0'
f.write_text(t.replace(viejo, nuevo)); print('tool0 reparentado')

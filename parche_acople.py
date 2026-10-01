"""Anade el acople cinematico de ADR-002 al modelo de la Estacion 1.

ADR-002 decidio que el agarre no se hace por rozamiento sino con un acople que
se conecta y desconecta. La razon esta escrita alli y conviene repetirla: coger
objetos por friccion en un simulador es inestable, la pieza vibra, se dispara o
atraviesa la mordaza, y es un problema del solucionador de contactos, no de
habilidad. Descubrirlo en octubre significaria perder la entrega.

En Gazebo Harmonic ese acople es el complemento DetachableJoint. Crea una union
fija entre un eslabon del robot y un modelo externo cuando llega un mensaje, y
la deshace cuando llega otro. No acopla al arrancar: espera la peticion, que es
justo lo que hace falta.

El eslabon padre es gripper_link y no link_3. La pieza la sujetan las mordazas,
de modo que es de ellas de quien tiene que colgar; si colgara del carro, abrir
la pinza no soltaria nada. El ciclo desacopla antes de abrir, que es el orden
correcto y el que evita que la pieza acompane el giro del servo.
"""
import pathlib
import sys

BLOQUE = '''
  <!-- ==================================================================
       Acople cinematico de la pieza, segun ADR-002.

       No acopla al arrancar. El nodo de la estacion pide el acople cuando
       cierra la mordaza sobre la pieza y lo deshace antes de abrirla.
  =================================================================== -->
  <xacro:arg name="pieza" default="pieza"/>
  <xacro:if value="${'$(arg pieza)' != ''}">
    <gazebo>
      <plugin filename="gz-sim-detachable-joint-system"
              name="gz::sim::systems::DetachableJoint">
        <parent_link>${p}gripper_link</parent_link>
        <child_model>$(arg pieza)</child_model>
        <child_link>cuerpo</child_link>
        <attach_topic>/station1/pieza/attach</attach_topic>
        <detach_topic>/station1/pieza/detach</detach_topic>
        <suppress_child_warning>true</suppress_child_warning>
      </plugin>
    </gazebo>
  </xacro:if>
'''


def main(ruta):
    p = pathlib.Path(ruta)
    t = p.read_text()
    if 'DetachableJoint' in t:
        print('el modelo ya tiene el acople, no se toca')
        return
    ancla = '  <xacro:station1_gazebo prefijo="${p}"/>\n'
    assert t.count(ancla) == 1, 'no encuentro la llamada a station1_gazebo'
    p.write_text(t.replace(ancla, ancla + BLOQUE))
    print('acople anadido a', ruta)


if __name__ == '__main__':
    main(sys.argv[1])

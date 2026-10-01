"""Anade al mundo la pieza, la bandeja de entrada y el soporte de traspaso.

Hasta ahora la linea movia informacion pero no materia: las estaciones se
avisaban correctamente pero no habia ningun objeto que coger. Esto lo arregla,
y de paso hace que los puntos de tarea signifiquen algo fisico.

Las alturas no son arbitrarias, se derivan de los puntos que ya estaban en
line_poses.yaml. El criterio es que **el punto de tarea es donde va el efector,
y la cara alta de la pieza queda 6 mm por debajo**.

  Bandeja: cara superior en z = 0.032. Una pieza de 12 mm apoyada tiene su
  centro en 0.038 y su cara alta en 0.044, con el efector bajando a 0.050.

  Soporte del traspaso: cara superior en 0.042, con lo que la pieza depositada
  queda con su centro en 0.048 y el efector a 0.060, la cota de handoff_1_2.

Esos 6 mm de holgura no son un descuido y merecen explicacion, porque costaron
dos intentos fallidos.

El primero: con la pieza centrada en el propio punto de tarea, su volumen y el
de la mordaza se solapaban 6 mm. El contacto empujaba el brazo, el codo se
desviaba 0.10 rad y el controlador abortaba la trayectoria.

El segundo, con 2 mm de holgura, fallaba por una razon que no se ve en un
dibujo de planta: **la mordaza se abre girando 0.5 rad sobre el eje X**, y al
girar, la esquina inferior de su bloque baja 2.9 mm por debajo del efector. El
descenso a por la pieza se hace precisamente con la mordaza abierta, de modo
que esa esquina la golpeaba y la lanzaba fuera de la bandeja. Con 6 mm quedan
3.1 de holgura incluso con la mordaza abierta del todo.

Es una simplificacion de modelado y conviene decirla en la sustentacion: la
mordaza sujeta la pieza desde arriba y no por los lados. La retencion la hace
igualmente el acople cinematico de ADR-002, no el contacto, de modo que la
holgura no cambia nada del comportamiento.
"""
import pathlib
import sys

PIEZA = 0.020, 0.020, 0.012      # una pieza de 20 x 20 x 12 mm
MASA = 0.020                     # 20 g

ixx = MASA * (PIEZA[1] ** 2 + PIEZA[2] ** 2) / 12
iyy = MASA * (PIEZA[0] ** 2 + PIEZA[2] ** 2) / 12
izz = MASA * (PIEZA[0] ** 2 + PIEZA[1] ** 2) / 12

MODELOS = """
    <!-- ================================================================
         Bandeja de entrada de la Estacion 1.

         Cara superior en z = 0.032. Con una pieza de 12 mm apoyada encima,
         su centro queda en 0.038 y su cara alta en 0.044, 6 mm por debajo del
         efector cuando este baja al punto de tarea. Las dimensiones en planta
         son las de input_tray en line_poses.yaml.
    ================================================================= -->
    <model name="bandeja_e1">
      <static>true</static>
      <pose>-0.180 0.100 0.016 0 0 0</pose>
      <link name="superficie">
        <collision name="collision">
          <geometry><box><size>0.100 0.100 0.032</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>0.100 0.100 0.032</size></box></geometry>
          <material>
            <ambient>0.72 0.58 0.38 1</ambient>
            <diffuse>0.72 0.58 0.38 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- ================================================================
         Soporte del punto de traspaso a la Estacion 2.

         Cara superior en z = 0.042, de modo que la pieza depositada queda
         con su centro en 0.048 y el efector en 0.060, la cota de handoff_1_2.
         Es el punto fijo de referencia que la guia exige entre las dos
         estaciones.
    ================================================================= -->
    <model name="soporte_traspaso_1_2">
      <static>true</static>
      <pose>0.250 0.000 0.021 0 0 0</pose>
      <link name="superficie">
        <collision name="collision">
          <geometry><box><size>0.060 0.060 0.042</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>0.060 0.060 0.042</size></box></geometry>
          <material>
            <ambient>0.35 0.40 0.45 1</ambient>
            <diffuse>0.35 0.40 0.45 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- ================================================================
         La pieza. Un componente electronico de 20 x 20 x 12 mm y 20 g.

         El rozamiento alto no es lo que la sujeta: la retencion la hace el
         acople cinematico de ADR-002. Esta aqui para que la pieza no resbale
         sobre la bandeja ni sobre el soporte una vez depositada.
    ================================================================= -->
    <model name="pieza">
      <pose>-0.180 0.100 0.038 0 0 0</pose>
      <link name="cuerpo">
        <inertial>
          <mass>%.4f</mass>
          <inertia>
            <ixx>%.3e</ixx><iyy>%.3e</iyy><izz>%.3e</izz>
            <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
          </inertia>
        </inertial>
        <collision name="collision">
          <geometry><box><size>%.3f %.3f %.3f</size></box></geometry>
          <surface>
            <friction><ode><mu>0.9</mu><mu2>0.9</mu2></ode></friction>
          </surface>
        </collision>
        <visual name="visual">
          <geometry><box><size>%.3f %.3f %.3f</size></box></geometry>
          <material>
            <ambient>0.80 0.15 0.15 1</ambient>
            <diffuse>0.80 0.15 0.15 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

""" % (MASA, ixx, iyy, izz, PIEZA[0], PIEZA[1], PIEZA[2],
       PIEZA[0], PIEZA[1], PIEZA[2])


def main(ruta):
    p = pathlib.Path(ruta)
    t = p.read_text()
    if 'name="pieza"' in t:
        print('el mundo ya tiene la pieza, no se toca')
        return
    ancla = '  </world>'
    assert t.count(ancla) == 1, 'no encuentro el cierre del mundo'
    p.write_text(t.replace(ancla, MODELOS + ancla))
    print('anadidos bandeja, soporte y pieza a', ruta)


if __name__ == '__main__':
    main(sys.argv[1])

"""Inserta la capa de ros2_control en el URDF de la Estacion 1.
Se aplica sobre la copia de trabajo de ~/E2_propuesta, nunca sobre el repo."""
import re, sys, pathlib

f = pathlib.Path(sys.argv[1])
t = f.read_text()

inc_viejo = '  <xacro:include filename="$(find station1_description)/urdf/station1.gazebo.xacro"/>\n'
inc_nuevo = (inc_viejo +
             '  <xacro:include filename="$(find station1_description)/urdf/station1.ros2_control.xacro"/>\n'
             '\n'
             '  <!-- Con control:=false el modelo se puede abrir en RViz sin levantar el\n'
             '       controller_manager, que es como se reviso el arbol TF en la Entrega 1. -->\n'
             '  <xacro:arg name="control" default="true"/>\n'
             '  <xacro:arg name="controladores"\n'
             '             default="$(find station1_description)/config/station1_controllers.yaml"/>\n')
assert t.count(inc_viejo) == 1, 'no encuentro el include de gazebo.xacro'
t = t.replace(inc_viejo, inc_nuevo)

fin_viejo = '  <xacro:station1_gazebo prefijo="${p}"/>\n'
fin_nuevo = (fin_viejo +
             '\n'
             '  <xacro:if value="$(arg control)">\n'
             '    <xacro:station1_ros2_control prefijo="${p}"\n'
             '                                 controladores="$(arg controladores)"/>\n'
             '  </xacro:if>\n')
assert t.count(fin_viejo) == 1, 'no encuentro la llamada a station1_gazebo'
t = t.replace(fin_viejo, fin_nuevo)

f.write_text(t)
print('parcheado', f)

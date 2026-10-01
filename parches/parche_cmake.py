"""Instala tambien config/ en el share del paquete."""
import sys, pathlib
f = pathlib.Path(sys.argv[1]); t = f.read_text()
viejo = 'install(DIRECTORY urdf launch rviz\n'
nuevo = 'install(DIRECTORY urdf launch rviz config\n'
assert t.count(viejo) == 1
f.write_text(t.replace(viejo, nuevo)); print('parcheado', f)

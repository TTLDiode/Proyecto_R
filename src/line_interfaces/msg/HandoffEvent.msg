# Describe una pieza y su ubicacion en el momento del evento.

# Identificador unico de la pieza dentro de la corrida
uint32 part_id

# Categoria asignada por la Estacion 2.
# Vacio ("") mientras la pieza no ha sido clasificada.
string category

# Pose de la pieza en el momento del evento, en frame "world"
geometry_msgs/Pose pose

# Marca de tiempo del evento
builtin_interfaces/Time stamp

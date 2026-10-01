"""Cambia la condicion del acople de una comparacion de cadenas a un booleano.

La primera version condicionaba el complemento con ${'$(arg pieza)' != ''}, que
obligaba a lanzar con pieza:= vacio para desactivarlo, y ros2 launch rechaza un
argumento vacio. Un booleano aparte es lo idiomatico en xacro y deja el nombre
del modelo como un dato independiente de si el acople se carga o no.
"""
import pathlib
import sys

VIEJO_INICIO = '  <xacro:arg name="pieza" default="pieza"/>'
NUEVO = ('  <xacro:arg name="acople" default="true"/>\n'
         '  <xacro:arg name="pieza" default="pieza"/>\n'
         '  <xacro:if value="$(arg acople)">')


def main(ruta):
    p = pathlib.Path(ruta)
    lineas = p.read_text().splitlines(keepends=True)
    salida = []
    i = 0
    hecho = False
    while i < len(lineas):
        if lineas[i].rstrip('\n') == VIEJO_INICIO and not hecho:
            # la linea siguiente es el xacro:if con la comparacion de cadenas
            assert 'xacro:if' in lineas[i + 1], 'no encuentro el if del acople'
            salida.append(NUEVO + '\n')
            i += 2
            hecho = True
            continue
        salida.append(lineas[i])
        i += 1
    if not hecho:
        print('ya estaba cambiado o no se encontro el bloque')
        return
    p.write_text(''.join(salida))
    print('condicion del acople cambiada a booleano en', ruta)


if __name__ == '__main__':
    main(sys.argv[1])

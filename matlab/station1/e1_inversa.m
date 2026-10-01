function [q, ok, motivo] = e1_inversa(pos)
%E1_INVERSA  Cinematica inversa de la Estacion 1, en forma cerrada.
%
%   [q, ok, motivo] = E1_INVERSA([x y z])
%
%   q       [theta1 theta2 d3], vacio si el punto no es alcanzable
%   ok      true si hay solucion dentro de los limites
%   motivo  'ok' o la razon del rechazo
%
%   La altura se despeja sola, porque z no depende de las rotacionales:
%       d3 = d1 + d2 - z
%
%   El plano es el problema de dos barras, con dos soluciones simetricas
%   respecto de la recta que une el eje con la herramienta:
%       cos(t2) = (r^2 - a1^2 - a2^2) / (2 a1 a2)
%       t2      = +/- acos(...)                      codo arriba o abajo
%       t1      = atan2(y,x) - atan2(a2 sen t2, a1 + a2 cos t2)
%
%   ELECCION DE RAMA. Se prueba primero codo arriba y solo se pasa a codo
%   abajo si la primera no cumple los limites. No es arbitrario: sobre el
%   espacio alcanzable el 82 % de los puntos solo admite codo arriba, y de
%   los que admiten las dos, la de codo abajo exige theta1 cerca de 174
%   grados, con el brazo replegado hacia atras, junto al final de carrera y
%   a la zona donde la holgura con su pilar baja a 8.7 mm. La de codo
%   arriba es siempre la mas despejada.
%
%   Ver tambien E1_DIRECTA, E1_PRUEBA_CINEMATICA.

    p = e1_parametros();
    x = pos(1);  y = pos(2);  z = pos(3);
    q = [];  ok = false;

    % --- Prismatica, desacoplada ---
    d3 = p.z0 - z;
    if d3 < -1e-9 || d3 > p.d3_max + 1e-9
        motivo = 'altura fuera de la carrera';
        return
    end
    d3 = min(max(d3, 0), p.d3_max);      % recorta el redondeo en los topes

    % --- Plano de dos barras ---
    c2 = (x^2 + y^2 - p.a1^2 - p.a2^2) / (2*p.a1*p.a2);
    if abs(c2) > 1 + 1e-12
        motivo = 'radio fuera del alcance';
        return
    end
    c2 = min(max(c2, -1), 1);            % idem en el borde del anillo

    for codo = [+1, -1]
        t2 = codo * acos(c2);
        if abs(t2) > p.t2_max + 1e-12
            continue
        end
        t1 = atan2(y, x) - atan2(p.a2*sin(t2), p.a1 + p.a2*cos(t2));
        t1 = mod(t1 + pi, 2*pi) - pi;    % a (-pi, pi]

        % theta1 llega a 185 grados, de modo que hay que probar tambien la
        % vuelta completa antes de descartar el punto.
        for cand = [t1, t1 + 2*pi]
            if cand >= p.t1_min - 1e-12 && cand <= p.t1_max + 1e-12
                q = [cand, t2, d3];
                ok = true;
                motivo = 'ok';
                return
            end
        end
    end

    motivo = 'fuera de los limites de articulacion';
end

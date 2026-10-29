function [sol, motivo] = e2_dos_barras(x, y, l1, l2, desfase, rama)
%E2_DOS_BARRAS  Problema plano de dos barras, con desfase en la segunda.
%
%   Devuelve la primera [t1 t2] que cumpla los limites, o [] y el motivo.
%   El desfase sirve para la camara, que esta girada 7.3 grados respecto del
%   eslabon 2.

    p = e2_parametros();
    sol = [];
    switch rama
        case 'arriba', orden = +1;
        case 'abajo',  orden = -1;
        otherwise,     orden = [+1, -1];
    end

    c2 = (x^2 + y^2 - l1^2 - l2^2) / (2*l1*l2);
    if abs(c2) > 1 + 1e-12
        motivo = 'radio fuera del alcance';
        return
    end
    c2 = min(max(c2, -1), 1);

    for codo = orden
        psi = codo * acos(c2);
        t2 = psi - desfase;
        if abs(t2) > p.t2_max + 1e-12, continue, end
        t1 = atan2(y, x) - atan2(l2*sin(psi), l1 + l2*cos(psi));
        t1 = mod(t1 + pi, 2*pi) - pi;
        % theta1 llega a 210 grados: hay que probar tambien la vuelta.
        for cand = [t1, t1 + 2*pi]
            if cand >= p.t1_min - 1e-12 && cand <= p.t1_max + 1e-12
                sol = [cand, t2];
                motivo = 'ok';
                return
            end
        end
    end
    motivo = 'fuera de los limites de articulacion';
end

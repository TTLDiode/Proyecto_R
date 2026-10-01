function [T, pos] = e1_directa(q)
%E1_DIRECTA  Cinematica directa de la Estacion 1.
%
%   [T, pos] = E1_DIRECTA([theta1 theta2 d3])
%
%   T    matriz homogenea 4x4 de tool0 respecto de la base
%   pos  posicion de tool0, [x y z] en metros
%
%   Angulos en radianes, d3 en metros y positivo hacia abajo.
%
%   La forma cerrada equivalente es
%       x = a1 cos(t1) + a2 cos(t1+t2)
%       y = a1 sen(t1) + a2 sen(t1+t2)
%       z = d1 + d2 - d3
%   es decir, el plano depende solo de las rotacionales y la altura solo de
%   la prismatica. Los dos problemas estan desacoplados, y de ahi que la
%   inversa salga cerrada.

    p = e1_parametros();
    t1 = q(1);  t2 = q(2);  d3 = q(3);

    T = e1_dh(t1, p.d1, p.a1, 0) * ...
        e1_dh(t2, p.d2, p.a2, pi) * ...
        e1_dh(0,  d3,   0,    0);

    pos = T(1:3, 4).';
end

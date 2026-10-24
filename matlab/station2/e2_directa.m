function [T, pos] = e2_directa(q)
%E2_DIRECTA  Cinematica directa de la Estacion 2, en el frame de la base.
%
%   [T, pos] = E2_DIRECTA([theta1 theta2 d3])
%
%   Forma cerrada equivalente:
%       x = a1 cos(t1) + a2 cos(t1+t2)
%       y = a1 sen(t1) + a2 sen(t1+t2)
%       z = d1 + d2 - d3

    p = e2_parametros();
    T = e2_dh(q(1), p.d1, p.a1, 0) * ...
        e2_dh(q(2), p.d2, p.a2, pi) * ...
        e2_dh(0,    q(3), 0,    0);
    pos = T(1:3, 4).';
end

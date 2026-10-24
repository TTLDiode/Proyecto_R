function [q, ok, motivo] = e2_inversa(pos, rama)
%E2_INVERSA  Cinematica inversa de la Estacion 2, en forma cerrada.
%
%   [q, ok, motivo] = E2_INVERSA([x y z])
%   [q, ok, motivo] = E2_INVERSA([x y z], 'arriba' | 'abajo' | 'auto')
%
%   pos en el frame de la base. Para un punto del mundo: pos - p.base.
%
%       d3      = d1 + d2 - z
%       cos(t2) = (r^2 - a1^2 - a2^2) / (2 a1 a2)
%       t2      = +/- acos(...)                      codo arriba o abajo
%       t1      = atan2(y,x) - atan2(a2 sen t2, a1 + a2 cos t2)
%
%   RAMAS. Con 'auto' se prueba codo arriba y luego codo abajo. Los dos
%   traspasos caen en ramas distintas: el 1-2, (-0.250, 0) en la base, solo
%   admite codo arriba (codo abajo pediria theta1 = 211.1 > 210); el 2-3,
%   (0.220, 0), solo admite codo abajo (codo arriba pediria theta1 = -39.4).

    if nargin < 2, rama = 'auto'; end
    p = e2_parametros();
    q = [];  ok = false;

    d3 = p.z0 - pos(3);
    if d3 < -1e-9 || d3 > p.d3_max + 1e-9
        motivo = 'altura fuera de la carrera';
        return
    end
    d3 = min(max(d3, 0), p.d3_max);

    [sol, motivo] = e2_dos_barras(pos(1), pos(2), p.a1, p.a2, 0, rama);
    if isempty(sol), return, end
    q = [sol, d3];
    ok = true;
end

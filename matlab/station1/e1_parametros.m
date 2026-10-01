function p = e1_parametros()
%E1_PARAMETROS  Constantes cinematicas de la Estacion 1.
%
%   Todas salen del ensamble CAD y coinciden con las de
%   src/station1_description/urdf/parametros.xacro. Si una cambia alli,
%   cambia aqui: los dos archivos describen la misma maquina.
%
%   Tabla de Denavit-Hartenberg, convencion estandar
%   A_i = Rz(theta_i) Tz(d_i) Tx(a_i) Rx(alpha_i)
%
%       i   theta      d (m)     a (m)    alpha
%       1   theta1 *   0.173     0.180    0
%       2   theta2 *  -0.053     0.140    pi
%       3   0          d3 *      0        0
%
%   alpha2 = pi voltea el eje z del sistema 2 hacia abajo, de modo que d3
%   positivo desciende sin necesidad de meter un signo a mano.
%
%   d2 es negativo y no tiene nada de raro: es el salto vertical desde el
%   plano del eje de theta1 hasta la herramienta con la prismatica arriba,
%   0.097 - 0.115 - 0.035 = -0.053 m.

    p.a1 = 0.180;               % eslabon 1, distancia entre ejes
    p.a2 = 0.140;               % eslabon 2, distancia entre ejes
    p.d1 = 0.173;               % altura del eje de theta1 sobre la base
    p.d2 = -0.053;              % del plano del brazo a la herramienta, d3 = 0

    p.t1_min = deg2rad(-15);    % el recorrido util llega a 131 grados; el
    p.t1_max = deg2rad(185);    % final de carrera dispara a 180
    p.t2_max = deg2rad(135);    % simetrico
    p.d3_max = 0.084;           % carrera de diseno 80 mm, sobran 4

    % Derivadas, para no repetir cuentas
    p.z0    = p.d1 + p.d2;                  % 0.120 m, altura de tool0 con d3 = 0
    p.r_max = p.a1 + p.a2;                  % 0.320 m, brazo extendido
    p.r_min = sqrt(p.a1^2 + p.a2^2 + ...    % 0.128 m, lo impone el tope del codo
                   2*p.a1*p.a2*cos(p.t2_max));
end

function p = e2_parametros()
%E2_PARAMETROS  Constantes cinematicas de la Estacion 2 (TRIADA).
%
%   Salen del ensamble CAD (docs/estacion2/urdf-datos-cad.md) y coinciden con
%   src/station2_description/urdf/parametros.xacro y con
%   src/station2_control/station2_control/cinematica.py.
%
%   Tabla de Denavit-Hartenberg, convencion estandar
%   A_i = Rz(theta_i) Tz(d_i) Tx(a_i) Rx(alpha_i)
%
%       i   theta      d (m)     a (m)    alpha
%       1   theta1 *   0.114     0.160    0
%       2   theta2 *   0.006     0.140    pi
%       3   0          d3 *      0        0
%
%   d1 es la altura del eje de theta1 sobre el suelo. d2 lleva de ese plano a
%   tool0 con d3 = 0: el eslabon 2 esta 42 mm por encima y el carro cuelga
%   36 mm por debajo (0.114 + 0.042 - 0.036 = 0.120). alpha2 = pi voltea el
%   eje z para que d3 positivo descienda.

    p.a1 = 0.160;               % eslabon 1, distancia entre ejes
    p.a2 = 0.140;               % eslabon 2, codo a vastago
    p.d1 = 0.114;               % altura del eje de theta1
    p.d2 = 0.006;               % del plano de theta1 a tool0, d3 = 0

    p.t1_min = deg2rad(-30);
    p.t1_max = deg2rad(210);
    p.t2_max = deg2rad(135);    % simetrico
    p.d3_max = 0.080;           % carrera del husillo Tr8

    % Base de la estacion en el mundo (line_bringup/config/line_poses.yaml)
    p.base = [0.500, 0.000, 0.000];

    % Camara (Camera Module 3 Wide) bajo el eslabon 2, frame del eslabon 2
    p.cam_x = 0.070;
    p.cam_y = 0.009;
    p.cam_z = 0.151;            % altura de la lente sobre el suelo
    p.cam_l = hypot(p.cam_x, p.cam_y);
    p.cam_fi = atan2(p.cam_y, p.cam_x);

    % Derivadas
    p.z0    = p.d1 + p.d2;                  % 0.120 m
    p.r_max = p.a1 + p.a2;                  % 0.300 m
    p.r_min = sqrt(p.a1^2 + p.a2^2 + ...    % 0.116 m, tope del codo
                   2*p.a1*p.a2*cos(p.t2_max));
end

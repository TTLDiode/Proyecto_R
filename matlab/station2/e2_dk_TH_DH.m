%% Contents
% Cinematica directa de la Estacion 2 (TRIADA, SCARA RRP) con DH y grafica
% Adaptado del codigo de clase ri_dk_ms3r_TH_DH.m (Robotica Industrial, EIA,
% docente David Rozo Osorio): misma transformada T_DH y misma grafica; aqui
% solo cambian la tabla DH y las configuraciones que se evaluan.

%% Direct Kinematic Model With DH and Graph
clear all
clc
close all

%% Tabla DH del robot (convencion del codigo de clase, DH modificado)
% Cada fila: DH = [a_ij alpha_ij s_j theta_j], en mm y rad.
%   fila 1: {0}->{1}  eje de theta1 a 114 mm del suelo
%   fila 2: {1}->{2}  eslabon 1 de 160 mm, codo 6 mm mas arriba (d2)
%   fila 3: {2}->{3}  eslabon 2 de 140 mm; alpha = pi voltea z hacia abajo,
%                     asi que d3 positiva baja la herramienta
%   fila 4: {3}->{T}  herramienta (tool0, puntas de los dedos), sin desfase
% Es la misma cadena que la tabla DH estandar de la Entrega 2
% (d1 = 0.114, a1 = 0.160; d2 = 0.006, a2 = 0.140, alpha2 = pi; d3).
dh_robot = @(th1, th2, d3) [ ...
    0    0    114  th1;
    160  0      6  th2;
    140  pi    d3    0;
    0    0      0    0];

% Base de la estacion en el mundo (line_poses.yaml), mm
BASE = [500; 0; 0];

% Configuraciones a evaluar: las de la comparacion con MoveIt
% (resultados/estacion2/comparacion_moveit_vs_propia.md), q = [th1 th2 d3]
nombres = {'traspaso_12', 'inspeccion', 'traspaso_23', 'libre_1', 'libre_2'};
Q = [148.90  67.28 60;
      45.95  96.67 60;
      39.40 -85.90 60;
      87.45  52.46 40;
       5.77  67.28 20];

fprintf('%-12s %8s %8s %6s   %10s %10s %10s\n', 'punto', 'th1', 'th2', 'd3', ...
        'x mundo', 'y mundo', 'z mundo');
for k = 1:size(Q,1)
    DH = dh_robot(deg2rad(Q(k,1)), deg2rad(Q(k,2)), Q(k,3));
    DoF = size(DH,1);
    for i = 1:DoF
        T_ij(:,:,i) = T_DH(DH(i,1),DH(i,2),DH(i,3),DH(i,4));
    end
    T(:,:,1) = T_ij(:,:,1);
    for i = 1:DoF-1
        T(:,:,i+1) = T(:,:,i)*T_ij(:,:,i+1);
    end
    p = T(1:3,4,DoF) + BASE;
    fprintf('%-12s %8.2f %8.2f %6.1f   %10.3f %10.3f %10.3f\n', nombres{k}, ...
            Q(k,1), Q(k,2), Q(k,3), p(1), p(2), p(3));
end
T_herramienta_ultimo = T(:,:,DoF)

%% Graph (ultima configuracion evaluada), igual que en el codigo de clase
hf=figure(1);
L=40;
plot3([0 L],[0 0],[0 0],'r','linewidth',2.5); hold on
plot3([0 0],[0 L],[0 0],'g','linewidth',2.5);
plot3([0 0],[0 0],[0 L],'b','linewidth',2.5);
axis equal
axis([-320 320 -320 320 0 200])
grid on
xlabel('X [mm]'); ylabel('Y [mm]'); zlabel('Z [mm]')
view(27,37)
Mp=[0,0,0,1; L,0,0,1; 0,L,0,1; 0,0,L,1];
for k=1:DoF
    for i=1:4
        Mp_T(i,:,k)=(T(:,:,k)*Mp(i,:)')';
    end
    plot3([Mp_T(1,1,k) Mp_T(2,1,k)],[Mp_T(1,2,k) Mp_T(2,2,k)],[Mp_T(1,3,k) Mp_T(2,3,k)],'r','linewidth',2)
    plot3([Mp_T(1,1,k) Mp_T(3,1,k)],[Mp_T(1,2,k) Mp_T(3,2,k)],[Mp_T(1,3,k) Mp_T(3,3,k)],'g','linewidth',2)
    plot3([Mp_T(1,1,k) Mp_T(4,1,k)],[Mp_T(1,2,k) Mp_T(4,2,k)],[Mp_T(1,3,k) Mp_T(4,3,k)],'b','linewidth',2)
end
% eslabones
P = [0 0 0; squeeze(T(1:3,4,:))'];
plot3(P(:,1),P(:,2),P(:,3),'k-o','linewidth',1.5)
title('Estacion 2 - cinematica directa (DH)')

%% Transform tipo Denavit-Hartemberg (la del codigo de clase)
function T_ij=T_DH(a_ij,alpha_ij,s_i,theta_i)

T_ij =[               cos(theta_i),              -sin(theta_i),              0,               a_ij;
    cos(alpha_ij)*sin(theta_i), cos(alpha_ij)*cos(theta_i), -sin(alpha_ij), -s_i*sin(alpha_ij);
    sin(alpha_ij)*sin(theta_i), sin(alpha_ij)*cos(theta_i),  cos(alpha_ij),  s_i*cos(alpha_ij);
    0,                          0,              0,                  1];
end

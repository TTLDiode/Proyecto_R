%% Contents
% Cinematica inversa (metodo geometrico) y directa de la Estacion 2, con
% trayectoria. Adaptado del codigo de clase ri_dk_ik_ms3r_trajectory.m
% (Robotica Industrial, EIA, docente David Rozo Osorio): mismo calculo de th2
% por la ley de cosenos y de th1 resolviendo el sistema A*X = B. El robot es
% un SCARA RRP: las dos rotacionales se resuelven en el plano (el 3R de clase
% sin la tercera barra, porque la orientacion no es libre) y la prismatica
% sale de la altura: d3 = d1 + d2 - z.

clear all
clc
close all

%% Parameters (mm)
r1 = 160;          % eslabon 1
r2 = 140;          % eslabon 2
d1 = 114;          % eje de theta1 sobre el suelo
d2 = 6;            % del plano de theta1 a la herramienta con d3 = 0
BASE = [500, 0];   % base de la estacion en el mundo (line_poses.yaml)

%% 1) Puntos de la comparacion con MoveIt (frame world, mm) y rama del codo
nombres = {'traspaso_12', 'inspeccion', 'traspaso_23', 'libre_1', 'libre_2'};
P = [250   0  60;
     500 200  60;
     720   0  60;
     400 250  80;
     700 150 100];
codo = [+1 +1 -1 +1 +1];   % +1 codo arriba (th2 > 0), -1 codo abajo
% Soluciones de MoveIt (KDL) de la misma tabla, para comparar
Q_moveit = [148.90  67.29 60.0;
             45.95  96.67 60.0;
             39.40 -85.90 60.0;
            136.15 -52.46 40.0;
             67.97 -67.28 20.0];

fprintf('%-12s %22s %8s %8s %6s %10s\n', 'punto', 'objetivo (mm)', 'th1', 'th2', ...
        'd3', 'cierre mm');
for k = 1:size(P,1)
    [th, ok] = ik_scara(P(k,:) - [BASE 0], r1, r2, d1, d2, codo(k));
    p = dk_scara(th, r1, r2, d1, d2) + [BASE 0];
    fprintf('%-12s (%6.1f %6.1f %5.1f) %8.2f %8.2f %6.1f %10.2e\n', nombres{k}, ...
            P(k,:), rad2deg(th(1)), rad2deg(th(2)), th(3), norm(p - P(k,:)));
end

fprintf('\nContraste con MoveIt (la otra rama del codo, cuando MoveIt la elige)\n');
for k = 1:size(P,1)
    rama = sign(Q_moveit(k,2));
    th = ik_scara(P(k,:) - [BASE 0], r1, r2, d1, d2, rama);
    dq = abs([rad2deg(th(1:2)) th(3)] - Q_moveit(k,:));
    fprintf('%-12s propia %8.2f %8.2f %6.1f | MoveIt %8.2f %8.2f %6.1f | max dif %.3f\n', ...
            nombres{k}, rad2deg(th(1)), rad2deg(th(2)), th(3), Q_moveit(k,:), max(dq));
end

%% 2) Trayectoria recta del traspaso 1-2 al puesto de inspeccion (como en clase)
NumPos = 100;
Rx = linspace(250, 500, NumPos) - BASE(1);
Ry = linspace(0, 200, NumPos) - BASE(2);
Rz = 80*ones(1, NumPos);
for i = 1:NumPos
    TH(i,:) = ik_scara([Rx(i) Ry(i) Rz(i)], r1, r2, d1, d2, +1);
end
th1 = TH(:,1); th2 = TH(:,2);
lx = r1*cos(th1) + r2*cos(th1+th2);
ly = r1*sin(th1) + r2*sin(th1+th2);
fprintf('\nTrayectoria: error maximo de la directa sobre la recta %.2e mm\n', ...
        max(hypot(lx - Rx', ly - Ry')));

%% Animation (igual que en el codigo de clase)
p1x = r1*cos(th1); p1y = r1*sin(th1);
fig1 = figure(1);
hb1 = plot(0,0,'linewidth',2); hold on
hb2 = plot(0,0,'linewidth',2);
hbt = plot(0,0,'k-.','linewidth',1);
axis equal
axis([-1.1*(r1+r2) 1.1*(r1+r2) -0.5*(r1+r2) 1.1*(r1+r2)])
grid on
xlabel('X base [mm]'); ylabel('Y base [mm]')
for i = 1:NumPos
    set(hb1,'xdata',[0,p1x(i)],'ydata',[0,p1y(i)]);
    set(hb2,'xdata',[p1x(i),lx(i)],'ydata',[p1y(i),ly(i)]);
    set(hbt,'xdata',lx(1:i),'ydata',ly(1:i))
    pause(0.0125)
end

%% Inversa geometrica (frame de la base)
function [th, ok] = ik_scara(p, r1, r2, d1, d2, codo)
    rx = p(1); ry = p(2); rz = p(3);
    num = rx^2 + ry^2 - r1^2 - r2^2;
    dem = 2*r1*r2;
    c2 = num/dem;
    ok = abs(c2) <= 1;
    th2 = codo*acos(max(-1, min(1, c2)));       % 2a solucion: el otro signo
    A = [r1 + r2*cos(th2), -r2*sin(th2);
         r2*sin(th2),       r1 + r2*cos(th2)];
    B = [rx; ry];
    X = A\B;
    th1 = atan2(X(2,1), X(1,1));
    d3 = d1 + d2 - rz;
    th = [th1, th2, d3];
end

%% Directa (para cerrar el lazo)
function p = dk_scara(th, r1, r2, d1, d2)
    p = [r1*cos(th(1)) + r2*cos(th(1)+th(2)), ...
         r1*sin(th(1)) + r2*sin(th(1)+th(2)), ...
         d1 + d2 - th(3)];
end

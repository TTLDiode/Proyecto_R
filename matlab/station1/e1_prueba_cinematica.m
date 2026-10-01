function e1_prueba_cinematica()
%E1_PRUEBA_CINEMATICA  Banco de pruebas de la cinematica de la Estacion 1.
%
%   Comprueba tres cosas:
%     1. Que directa e inversa cierran sobre todo el espacio alcanzable.
%     2. Que los puntos imposibles se rechazan, y por el motivo correcto.
%     3. Que los seis puntos de tarea salen con los valores documentados.
%
%   No basta con probar la inversa contra puntos conocidos: hay que barrer
%   el volumen, porque los fallos de una inversa cerrada aparecen en los
%   bordes, no en el centro.

    p = e1_parametros();
    fprintf('\n== Cierre directa-inversa sobre el espacio alcanzable ==\n');

    peor = 0;  n_ok = 0;  n_no = 0;  arriba = 0;
    for r = p.r_min : 0.002 : p.r_max
        for ang = deg2rad(-15) : deg2rad(2) : deg2rad(185)
            for z = (p.z0 - p.d3_max) : 0.006 : p.z0
                objetivo = [r*cos(ang), r*sin(ang), z];
                [q, ok] = e1_inversa(objetivo);
                if ok
                    n_ok = n_ok + 1;
                    arriba = arriba + (q(2) > 0);
                    peor = max(peor, norm(e1_directa(q) * [0;0;0;1] ...
                                          - [objetivo.'; 1]));
                else
                    n_no = n_no + 1;
                end
            end
        end
    end

    fprintf('  poses ensayadas        %d\n', n_ok + n_no);
    fprintf('  resueltas              %d  (%.1f %%)\n', n_ok, ...
            100*n_ok/(n_ok + n_no));
    fprintf('  error maximo de cierre %.2e m\n', peor);
    fprintf('  codo arriba            %.1f %%\n', 100*arriba/n_ok);

    fprintf('\n== Rechazo de puntos imposibles ==\n');
    casos = { 'mas alla del alcance',    [ 0.340,  0.000, 0.080] ;
              'dentro del radio muerto', [ 0.050,  0.000, 0.080] ;
              'demasiado alto',          [ 0.250,  0.000, 0.150] ;
              'demasiado bajo',          [ 0.250,  0.000, 0.020] ;
              'detras, a -60 grados',    [ 0.150, -0.260, 0.080] };
    for k = 1:size(casos, 1)
        [~, ok, motivo] = e1_inversa(casos{k, 2});
        estado = 'rechaza';
        if ok, estado = 'RESUELVE'; end
        fprintf('  %-26s %-9s %s\n', casos{k, 1}, estado, motivo);
    end

    fprintf('\n== Puntos de tarea ==\n');
    tarea = { 'Centro de la bandeja',   [-0.180, 0.100, 0.050] ;
              'Esquina (-,-)',          [-0.230, 0.050, 0.050] ;
              'Esquina (-,+)',          [-0.230, 0.150, 0.050] ;
              'Esquina (+,-)',          [-0.130, 0.050, 0.050] ;
              'Esquina (+,+)',          [-0.130, 0.150, 0.050] ;
              'Traspaso a Estacion 2',  [ 0.250, 0.000, 0.060] };
    fprintf('  %-24s %8s %8s %9s  %s\n', 'punto', 'theta1', 'theta2', 'd3 (mm)', 'codo');
    for k = 1:size(tarea, 1)
        [q, ok] = e1_inversa(tarea{k, 2});
        if ~ok
            fprintf('  %-24s  NO ALCANZABLE\n', tarea{k, 1});
            continue
        end
        codo = 'abajo';
        if q(2) > 0, codo = 'arriba'; end
        fprintf('  %-24s %8.1f %8.1f %9.1f  %s\n', tarea{k, 1}, ...
                rad2deg(q(1)), rad2deg(q(2)), q(3)*1000, codo);
    end
    fprintf('\n');
end

function e2_prueba_cinematica()
%E2_PRUEBA_CINEMATICA  Banco de pruebas de la cinematica de la Estacion 2.
%
%   1. Directa e inversa cierran sobre todo el espacio alcanzable.
%   2. Los puntos imposibles se rechazan, y por el motivo correcto.
%   3. Los puntos de tarea salen con los valores documentados.
%   4. La camara llega encima del puesto de inspeccion.
%
%   Misma prueba que src/station2_control/station2_control/prueba_cinematica.py.

    p = e2_parametros();
    fprintf('\n== Cierre directa-inversa sobre el espacio alcanzable ==\n');

    peor = 0;  n_ok = 0;  n_no = 0;  arriba = 0;
    for t1 = p.t1_min : deg2rad(4) : p.t1_max
        for t2 = -p.t2_max : deg2rad(4) : p.t2_max
            for d3 = 0 : 0.010 : p.d3_max
                [~, objetivo] = e2_directa([t1, t2, d3]);
                for rama = {'arriba', 'abajo'}
                    [q, ok] = e2_inversa(objetivo, rama{1});
                    if ok
                        n_ok = n_ok + 1;
                        arriba = arriba + (q(2) > 0);
                        [~, alcanzado] = e2_directa(q);
                        peor = max(peor, norm(alcanzado - objetivo));
                    else
                        n_no = n_no + 1;
                    end
                end
            end
        end
    end
    fprintf('  inversas ensayadas     %d\n', n_ok + n_no);
    fprintf('  resueltas              %d  (la otra rama puede salir de limites)\n', n_ok);
    fprintf('  error maximo de cierre %.2e m\n', peor);

    fprintf('\n== Rechazo de puntos imposibles (frame de la base) ==\n');
    casos = { 'mas alla del alcance',  [ 0.320,  0.000, 0.080] ;
              'dentro del radio muerto', [ 0.050,  0.000, 0.080] ;
              'demasiado alto',        [ 0.200,  0.100, 0.150] ;
              'demasiado bajo',        [ 0.200,  0.100, 0.020] ;
              'detras, a -90 grados',  [ 0.000, -0.250, 0.080] };
    for k = 1:size(casos, 1)
        [~, ok, motivo] = e2_inversa(casos{k, 2});
        estado = 'rechaza';
        if ok, estado = 'RESUELVE'; end
        fprintf('  %-26s %-9s %s\n', casos{k, 1}, estado, motivo);
    end

    fprintf('\n== Puntos de tarea (frame world) ==\n');
    tarea = { 'Traspaso 1-2',          [0.250, 0.000, 0.060], 'arriba' ;
              'Puesto de inspeccion',  [0.500, 0.200, 0.060], 'auto' ;
              'Traspaso 2-3',          [0.720, 0.000, 0.060], 'abajo' };
    fprintf('  %-24s %8s %8s %9s  %s\n', 'punto', 'theta1', 'theta2', 'd3 (mm)', 'codo');
    for k = 1:size(tarea, 1)
        [q, ok] = e2_inversa(tarea{k, 2} - p.base, tarea{k, 3});
        if ~ok
            fprintf('  %-24s  NO ALCANZABLE\n', tarea{k, 1});
            continue
        end
        codo = 'abajo';
        if q(2) > 0, codo = 'arriba'; end
        fprintf('  %-24s %8.2f %8.2f %9.1f  %s\n', tarea{k, 1}, ...
                rad2deg(q(1)), rad2deg(q(2)), q(3)*1000, codo);
    end

    fprintf('\n== Camara sobre el puesto de inspeccion ==\n');
    xy = [0.500, 0.200] - p.base(1:2);
    [q, ok, motivo] = e2_inversa_camara(xy);
    if ok
        xc = p.a1*cos(q(1)) + p.cam_l*cos(q(1) + q(2) + p.cam_fi);
        yc = p.a1*sin(q(1)) + p.cam_l*sin(q(1) + q(2) + p.cam_fi);
        fprintf('  theta1 %.2f  theta2 %.2f  lente en (%.4f, %.4f), error %.2e m\n', ...
                rad2deg(q(1)), rad2deg(q(2)), xc, yc, norm([xc yc] - xy));
    else
        fprintf('  NO ALCANZABLE: %s\n', motivo);
    end
    xy = [0.250, 0.000] - p.base(1:2);
    [~, ok, motivo] = e2_inversa_camara(xy);
    fprintf('  camara sobre el traspaso 1-2: %s (%s)\n', ...
            ternario(ok, 'alcanza', 'no alcanza'), motivo);
    fprintf('\n');
end

function s = ternario(c, a, b)
    if c, s = a; else, s = b; end
end

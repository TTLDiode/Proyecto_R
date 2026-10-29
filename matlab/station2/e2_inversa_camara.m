function [q, ok, motivo] = e2_inversa_camara(xy, rama)
%E2_INVERSA_CAMARA  Pose que pone la lente de la camara sobre (x, y).
%
%   Mismo problema de dos barras, con la segunda medida hasta la lente
%   (70.6 mm, girada 7.3 grados). d3 queda en 0 para que la mordaza no
%   entre en la imagen. xy en el frame de la base.

    if nargin < 2, rama = 'auto'; end
    p = e2_parametros();
    q = [];  ok = false;
    [sol, motivo] = e2_dos_barras(xy(1), xy(2), p.a1, p.cam_l, p.cam_fi, rama);
    if isempty(sol), return, end
    q = [sol, 0];
    ok = true;
end

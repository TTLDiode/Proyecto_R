function A = e1_dh(theta, d, a, alpha)
%E1_DH  Matriz homogenea de un eslabon en la convencion estandar.
%
%   A = Rz(theta) * Tz(d) * Tx(a) * Rx(alpha)

    ct = cos(theta);  st = sin(theta);
    ca = cos(alpha);  sa = sin(alpha);

    A = [ ct, -st*ca,  st*sa, a*ct ;
          st,  ct*ca, -ct*sa, a*st ;
           0,     sa,     ca,    d ;
           0,      0,      0,    1 ];
end

function [Ed, Eb] = Errors(Xcw, Ycw, Xccw, Yccw, L, b);

betax = (Xcw - Xccw)/(-4*L);
betay = (Ycw + Yccw)/(-4*L);

beta = (betax + betay) / 2

R = L/2 / sin(beta/2);

Ed = (R+b/2)/(R-b/2);

alfax = (Xcw + Xccw)/(-4*L) * 180/pi;
alfay = (Ycw - Yccw)/(-4*L) * 180/pi;

alfa = (alfax+alfay)/2;

Eb = 90/(90-alfa);

end
k0 = [16, 10];
d = [0.75, 0.65, 0.55, 0.45, 0.35, 0.25, 0.15];
irout = [];

k = lsqcurvefit(irdist, k0 ,d , irout)
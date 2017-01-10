clear all; clc; close all;

data = importdata('ir1_vel0.1.log');
dist = data(data(:,1) > 0, 1) * 1e-02;

% irout = data(data(:,1) > 0, [3,4,5])

iroutl = data(data(:,1) > 0, 3);
iroutm = data(data(:,1) > 0, 4);
iroutr = data(data(:,1) > 0, 5);

kl = lsqcurvefit(@irdist, [16, 10], dist, iroutl)
km = lsqcurvefit(@irdist, [16, 10], dist, iroutm)
kr = lsqcurvefit(@irdist, [16, 10], dist, iroutr)

d = [0.75, 0.65, 0.55, 0.45, 0.35, 0.25, 0.15];

figure; hold on;
plot(dist, iroutl, 'b+');
plot(dist, iroutm, 'g+');
plot(dist, iroutr, 'r+');
plot(d, irdist(kl,d), 'b-');
plot(d, irdist(km,d), 'g-');
plot(d, irdist(kr,d), 'r-');
legend('Left, measured','Middle, measured','Right, measured','Left, estimated','Middle, estimated','Right, estimated','Location','NorthEast');
axis([0.1, 0.8, 40, 160]);
xlabel('Distance [m]'); ylabel('IR Sensor Output');
title('IR Sensor Measurements');
hold off;

figure; hold on;
plot(dist, iroutl, 'b+');
plot(d, irdist(kl,d), 'b-');
legend('Left, measured','Left, estimated','Location','NorthEast');
axis([0.1, 0.8, 40, 160]);
xlabel('Distance [m]'); ylabel('IR Sensor Output');
title('IR Sensor Measurements');
hold off;

figure; hold on;
plot(dist, iroutm, 'g+');
plot(d, irdist(km,d), 'g-');
legend('Middle, measured','Middle, estimated','Location','NorthEast');
axis([0.1, 0.8, 40, 160]);
xlabel('Distance [m]'); ylabel('IR Sensor Output');
title('IR Sensor Measurements');
hold off;

figure; hold on;
plot(dist, iroutr, 'r+');
plot(d, irdist(kr,d), 'r-');
legend('Right, measured','Right, estimated','Location','NorthEast');
axis([0.1, 0.8, 40, 160]);
xlabel('Distance [m]'); ylabel('IR Sensor Output');
title('IR Sensor Measurements');
hold off;
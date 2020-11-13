% 25 nutation
% px1 = -0.088;
% py1 = -1.868;
% px2 = 0.089;
% py2 = 2.395;
% t = 155.1052;

% 30 nutation
% px1 = 0.539;
% py1 = 0.877;
% px2 = -0.129;
% py2 = -1.98;
% t = 171.5660;

% 35 nutation
px1 = 0.311;
py1 = 2.188;
px2 = -0.117;
py2 = 0.058;
t = 196.1399;

% calculate table data

CBL = 0.95;

dist_sqr = (px1-px2)^2 + (py1-py2)^2;
dist = sqrt(dist_sqr);
dist_CBL = dist / CBL;
vel_CBL = dist_CBL / t;
vel_CBL_1000 = vel_CBL * 1000;
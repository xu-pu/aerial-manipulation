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
% px1 = 0.311;
% py1 = 2.188;
% px2 = -0.117;
% py2 = 0.058;
% t = 196.1399;


% labfloor 35 nutation
% px1 = 0.354 ;
% py1 = -2.745 ;
% px2 = 1.263 ;
% py2 = 1.822 ;
% t = 196.1399;

% % labfloor 30 nutation
% px1 = 0.414 ;
% py1 = -2.511 ;
% px2 = -0.063 ;
% py2 = 1.694 ;
% t = 171.5660;

% labfloor 25 nutation
% px1 = 0.464 ;
% py1 = -2.711 ;
% px2 = 0.501 ;
% py2 = 1.038 ;
% t = 155.1052;

% nominal 40 nutation 0.8379
% px1 = 0.496 ;
% py1 = -2.506 ;
% px2 = 0.457 ;
% py2 = -1.669 ;
% t = 166;

% nominal 35 nutation 1.1919
% px1 = 0.488 ;
% py1 = -2.148 ;
% px2 =  0.555 ;
% py2 =  -0.958 ;
% t = 155;

% % nominal 30 nutation 1.3640
% px1 = 0.244;
% py1 = -2.328;
% px2 = 0.246 ;
% py2 = -0.964 ;
% t = 163;

% nominal 25 nutation
% px1 = 0.483 ;
% py1 = -2.557 ;
% px2 = 0.703 ;
% py2 = -1.678 ;
% t = 155.1052;

% labfloor long
% px1 = -0.446 ;
% py1 = -2.909 ;
% px2 = 0.263 ;
% py2 = -0.073 ;
% t = 100;
% steps = 130;

% labfloor truncated
px1 = -0.446 ;
py1 = -2.909 ;
px2 = 0.048  ;
py2 = -0.676  ;
t = 100;
steps = 100;

% calculate table data

CBL = 0.95;

dist_sqr = (px1-px2)^2 + (py1-py2)^2;
dist = sqrt(dist_sqr);
dist_CBL = dist / CBL;
vel_CBL = dist_CBL / t;
vel_CBL_1000 = vel_CBL * 1000;

per_step = dist/steps;
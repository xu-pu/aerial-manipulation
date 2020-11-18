load('xy_path.mat');

dat = ts_xy.Data;
[a, b] = size(dat);
% 5205 points, select a range that looks like a circle
dat = dat(1000:5205,:); 
axis equal
plot(dat(:,1),dat(:,2));
axis equal
axis ([-1.5, 0.5, -2.5,-0.5]);
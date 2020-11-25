%roboticsAddons
%rosgenmsg('/home/sheep/am_ws/src/aerial_manip')


%rosmsg list
% 
bag = rosbag('2020-11-22-22-59-37.n30.6.bag');
bSel = select(bag,'Topic','/rnw/walking_state/session_1');

ts_xy = timeseries(bSel, 'ConeState.ContactPoint.X', 'ConeState.ContactPoint.Y');
ts_xy.Time = ts_xy.Time - min(ts_xy.Time);


%load('xy_path.mat');

dat = ts_xy.Data;
[a, b] = size(dat);
%dat = dat(:,:);
axis equal
plot(dat(:,1),dat(:,2));
axis equal
%axis ([-1.5, 0.5, -2.5,-0.5]);

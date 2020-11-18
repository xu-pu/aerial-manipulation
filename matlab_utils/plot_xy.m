%roboticsAddons
%rosgenmsg('/home/sheep/am_ws/src/aerial_manip')


%rosmsg list
% 
% bag = rosbag('2020-11-12-00-09-15.test.lap.2.bag');
% bSel = select(bag,'Topic','/rnw/walking_state/session_1');
% 
% ts_xy = timeseries(bSel, 'ConeState.ContactPoint.X', 'ConeState.ContactPoint.Y');
% ts_xy.Time = ts_xy.Time - min(ts_xy.Time);
% 

load('xy_path.mat');

dat = ts_xy.Data;
[a, b] = size(dat);
dat = dat(1000:5205,:);
axis equal
plot(dat(:,1),dat(:,2));
axis equal
axis ([-1.5, 0.5, -2.5,-0.5]);


%plot2d(ts_xy)
% 
% tiledlayout(3,1)
% 
% nexttile
% plot(ts_euler_z)
% set(gca,'xtick',[])
% set(gca,'xticklabel',{[]})
% nexttile
% plot(ts_euler_y)
% set(gca,'xtick',[])
% nexttile
% plot(ts_euler_x)
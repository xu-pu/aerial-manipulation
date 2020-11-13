%roboticsAddons
%rosgenmsg('/home/sheep/am_ws/src/aerial_manip')


%rosmsg list

% bag = rosbag('2020-10-05-11-04-57.table.n35t40.bag');
% bSel = select(bag,'Topic','/rnw/walking_state/session_1');
% ts_euler_z = timeseries(bSel, 'ConeState.EulerAngles.Z');
% ts_euler_y = timeseries(bSel, 'ConeState.EulerAngles.Y');
% ts_euler_x = timeseries(bSel, 'ConeState.EulerAngles.X');
% 
% ts_euler_y.Time = ts_euler_y.Time - min(ts_euler_z.Time);
% ts_euler_z.Time = ts_euler_z.Time - min(ts_euler_z.Time);
% ts_euler_x.Time = ts_euler_x.Time - min(ts_euler_x.Time);


load('data_table_30.mat');

max(ts_euler_z.Time)

tiledlayout(3,1)

nexttile
plot(ts_euler_z)
set(gca,'xtick',[])
set(gca,'xticklabel',{[]})
nexttile
plot(ts_euler_y)
set(gca,'xtick',[])
nexttile
plot(ts_euler_x)
bag = rosbag('2020-11-20-16-03-46.bag');
bSel = select(bag,'Topic','/rnw/cone_energy_diff');
ts_energy = timeseries(bSel, 'Value');

ts_energy.Time = ts_energy.Time - min(ts_energy.Time);

% ts_euler_y.Time = ts_euler_y.Time - min(ts_euler_z.Time);
% ts_euler_z.Time = ts_euler_z.Time - min(ts_euler_z.Time);
% ts_euler_x.Time = ts_euler_x.Time - min(ts_euler_x.Time);

plot(ts_energy);
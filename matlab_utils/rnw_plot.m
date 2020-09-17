bag = rosbag('2020-09-14-00-58-00.bag');
bSel = select(bag,'Topic','/rnw/cone_state');
msgStructs = readMessages(bSel,'DataFormat','struct');

xPoints = cellfun(@(m) min(double(m.ContactPoint.X),1),msgStructs);
yPoints = cellfun(@(m) double(m.ContactPoint.Y),msgStructs);
axis equal
plot(xPoints,yPoints)
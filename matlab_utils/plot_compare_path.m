load('contact_labfloor.mat');

[a, b] = size(dat);
x1 = dat(1,1);
y1 = dat(1,2);

for i=1:a
    dat(i,1) = dat(i,1) - x1;
    dat(i,2) = dat(i,2) - y1;
end

axis equal
plot(dat(:,2),dat(:,1));
axis equal
axis ([0, 4, -0.3,0.3]);


dat_x_floor = dat(:,2);
dat_y_floor = dat(:,1);
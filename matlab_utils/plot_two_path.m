load('path_compare.mat');

tiledlayout(2,1)

plot_length = 4;
plot_height = 0.3;

nexttile
plot(dat_x_floor,dat_y_floor);
axis ([0, plot_length, -plot_height, plot_height]);
axis equal
%set(gca,'xtick',[])
%set(gca,'xticklabel',{[]})
nexttile
plot(dat_x_foam, dat_y_foam);
axis ([0, plot_length, -plot_height, plot_height]);
axis equal
%set(gca,'xtick',[])

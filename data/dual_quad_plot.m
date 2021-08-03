set(0,'DefaultAxesFontSize',20)
set(0,'defaulttextfontsize',20)

figure('Position',[0 0 850 1080])

t_start = 60;
t_end = 120;

width_plt = 2
width_box = 2

ax = subplot(4,1,1)
plot(ke(:,1),ke(:,2),'LineWidth',width_plt)
xlim([t_start,t_end])
ylim([0.0,0.4])
xticklabels('')
yticks([0.1,0.2,0.3])
ax.LineWidth = width_box
ax = subplot(4,1,2)
plot(phi(:,1),phi(:,2),'LineWidth',width_plt)
xlim([t_start,t_end])
ylim([-0.8,0.8])
xticklabels('')
yticks([-0.5,0,0.5])
ax.LineWidth = width_box
ax = subplot(4,1,3)
plot(cable(:,1),cable(:,2),'LineWidth',width_plt)
xlim([t_start,t_end])
xticklabels('')
ax.LineWidth = width_box
ylim([105,135])
yticks([110,120,130])
%ylim([75,105])
%yticks([80,90,100])
ax = subplot(4,1,4)
plot(thrust(:,1),thrust(:,2),'LineWidth',width_plt)
xlim([t_start,t_end])
ylim([15,35])
yticks([20,25,30])
ax.LineWidth = width_box
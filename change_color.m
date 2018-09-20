clc;clear;close all;
filename = dir('*.fig');
for i = 1:size(filename,1)
figure = open(filename(i).name);
figure = gca;
line = get(gca,'Children')
line(2).Color =   [0    0.4470    0.7410];
line(1).Color = [0.8500    0.3250    0.0980];
set(figure,'Color','w');
savefig(filename(i).name);
close;
end
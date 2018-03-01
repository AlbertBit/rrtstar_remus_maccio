close all;
clear;
clc;

report_id= fopen(strcat('./../Datalog/object_size2.txt'));
report_struct=textscan(report_id,'%f %f %f %f %f');
fclose(report_id);
rep2=[ report_struct{1},report_struct{2}, report_struct{3},report_struct{4}, report_struct{5}];

report_id= fopen(strcat('./../Datalog/object_size4.txt'));
report_struct=textscan(report_id,'%f %f %f %f %f');
fclose(report_id);
rep4=[ report_struct{1},report_struct{2}, report_struct{3},report_struct{4}, report_struct{5}];

report_id= fopen(strcat('./../Datalog/object_size8.txt'));
report_struct=textscan(report_id,'%f %f %f %f %f');
fclose(report_id);
rep8=[ report_struct{1},report_struct{2}, report_struct{3},report_struct{4}, report_struct{5}];

%object_size	collision_rate	collision_probability	elapsed_time	path_size

plot(rep2(:,1).^3, rep2(:,2), '*-');
hold on;
grid on; 
plot(rep2(:,1).^3, rep2(:,3), '*-');
xlabel('Object volume');
legend('collision rate','collision probability');
title('Collision analysis, obstacle side length 2');

figure


plot(rep4(:,1).^3, rep4(:,2), '*-');
hold on;
grid on; 
plot(rep4(:,1).^3, rep4(:,3), '*-');
xlabel('Object volume');
legend('collision rate','collision probability');
title('Collision analysis, obstacle side length 4');

figure

plot(rep8(:,1).^3, rep8(:,2), '*-');
hold on;
grid on; 
plot(rep8(:,1).^3, rep8(:,3), '*-');
title('Collision analysis, obstacle side length 8');
xlabel('Object volume');
legend('collision rate','collision probability');

figure

plot(rep2(:,1).^3, rep2(:,4), '*-');
grid on; 
hold on;
plot(rep4(:,1).^3, rep4(:,4), '*-');
hold on;
plot(rep8(:,1).^3, rep8(:,4), '*-');
title('Timing analysis');
xlabel('Object volume');
ylabel('Completion time');
legend('Obstacle side length 2','Obstacle side length 4', 'Obstacle side length 8');

% figure
% 
% plot(rep2(:,1), rep2(:,5), '*-');
% hold on;
% plot(rep4(:,1), rep4(:,5), '*-');
% hold on;
% plot(rep8(:,1), rep8(:,5), '*-');
% title('Optimal path length (0--> no path)');
% xlabel('Object dimension');
% ylabel('Path length (number of nodes)');
% legend('Object side length 2','Object side length 4', 'Object side length 8');





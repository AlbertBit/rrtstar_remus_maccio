close all;
clear;
clc;

report_id= fopen(strcat('./../Datalog/obstacle_timings.txt'));
report_struct=textscan(report_id,'%f %f %f');
fclose(report_id);
rep=[ report_struct{1},report_struct{2}, report_struct{3}];

report_id= fopen(strcat('./../Datalog/obstacle_timings05.txt'));
report_struct=textscan(report_id,'%f %f %f');
fclose(report_id);
rep05=[ report_struct{1},report_struct{2}, report_struct{3}];

report_id= fopen(strcat('./../Datalog/obstacle_timings10.txt'));
report_struct=textscan(report_id,'%f %f %f');
fclose(report_id);
rep10=[ report_struct{1},report_struct{2}, report_struct{3}];

%object_size	collision_rate	elapsed_time path_size
plot(rep(:,1), rep(:,2), '*-');
hold on;
plot(rep05(:,1), rep05(:,2), '*-');
hold on;
plot(rep10(:,1), rep10(:,2), '*-');
hold on;
grid on;


title('Collision analysis over 30 simulations');
xlabel('Number of obstacles');
ylabel('Collision rate');
legend('Object side length 0', 'Object side length 0.5', 'Object side length 1');


figure
plot(rep(:,1), rep(:,3), '*-');
hold on;
plot(rep05(:,1), rep05(:,3), '*-');
hold on;
plot(rep10(:,1), rep10(:,3), '*-');
hold on;
grid on;
title('Timing analysis over 30 simulations');
xlabel('Number of obstacles');
ylabel('Completion time');
legend('Object side length 0', 'Object side length 0.5', 'Object side length 1');


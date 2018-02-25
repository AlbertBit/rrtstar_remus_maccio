close all;
clear;
clc;

report_id= fopen(strcat('./../Datalog/report.txt')); 
report_struct=textscan(report_id,'%f %f');
fclose(report_id);
rep=[ report_struct{1},report_struct{2}];

report_obs_id= fopen(strcat('./../Datalog/report_obstacle.txt')); 
report_obs_struct=textscan(report_obs_id,'%f %f');
fclose(report_obs_id);
rep_obs=[ report_obs_struct{1},report_obs_struct{2}];

report_n_obs_id= fopen(strcat('./../Datalog/report_avg_num_obstacle.txt')); 
report_n_obs_struct=textscan(report_n_obs_id,'%f %f %f');
fclose(report_n_obs_id);
rep_avg_obs=[ report_n_obs_struct{1},report_n_obs_struct{2},report_n_obs_struct{3}];

report_n_obs_var_size_id= fopen(strcat('./../Datalog/report_avg_num_obstacles_var_size.txt')); 
report_n_obs_var_size_struct=textscan(report_n_obs_var_size_id,'%f %f %f %f');
fclose(report_n_obs_var_size_id);
rep_avg_obs_obj_size=[ report_n_obs_var_size_struct{1},report_n_obs_var_size_struct{2},report_n_obs_var_size_struct{3},report_n_obs_var_size_struct{4} ];

subplot(3,1,1);
plot(rep(:,1),rep(:,2),'*');
title('number of collisions during sampling');
xlabel('size of the object x 0.1');
ylabel('collisions');

subplot(3,1,2);
plot(rep_obs(:,1),rep_obs(:,2),'*');
title('number of collisions during sampling');
xlabel('size of the obstacle x 0.1');
ylabel('collisions');

subplot(3,1,3);
plot(rep_avg_obs(:,1),rep_avg_obs(:,2),'*');
title('Completion times 10 simulations');
xlabel('number of random obstacles');
ylabel('time for completions [s]');

figure;
%number of obstacle ---- size of the object ---- total time

n = length(rep_avg_obs_obj_size);
l = 5;
for i=1:l:n-l+1

    stem3(rep_avg_obs_obj_size(i:i+l-1,1),rep_avg_obs_obj_size(i:i+l-1,2),rep_avg_obs_obj_size(i:i+l-1,3),'*');
    hold on;

end

title('Average completion time over 10 simulations');
xlabel('number of random obstacles');
ylabel('object size x 0.1');
zlabel('time for completions [s]');

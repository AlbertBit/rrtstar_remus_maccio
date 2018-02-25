clc;clear all;close all;

MEAN_TIMES=zeros(1,9);
for i=1:9
time_id= fopen(strcat('/home/nasa/Datalog/Planner/timeEvaluation/',int2str(i),'Time.txt')); 
timeStruc=textscan(time_id,'%f %f');fclose(time_id);
TIMES=[timeStruc{2}]; 
MEAN_TIMES(i)=mean(TIMES);
end

bar(MEAN_TIMES);xlabel('No. of Obstacles'); ylabel('Mean time(sec)'); 
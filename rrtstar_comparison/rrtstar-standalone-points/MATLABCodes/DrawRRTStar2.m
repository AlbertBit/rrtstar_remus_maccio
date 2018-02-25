clc;clear all;close all;

% Path 
% 4-> 21,22,23 check
path_id= fopen(strcat('./../Datalog/Planner/Time/Obstacle15/0optTraj.txt')); 
pathStruc=textscan(path_id,'%f %f %f');
fclose(path_id);
path=[ pathStruc{1},pathStruc{2},pathStruc{3}]; 
pathX=path(:,1);
pathY=path(:,2);
pathZ=path(:,3);
% Regions
reg_id= fopen(strcat('./../Datalog/Planner/Time/Obstacle15/0Regions.txt')); 
regStruc=textscan(reg_id,'%f %f %f %f %f %f');
fclose(reg_id);
regions=[ regStruc{1},regStruc{2},regStruc{3},regStruc{4},regStruc{5},regStruc{6}];
% Obstacle
obs_id= fopen(strcat('./../Datalog/0Regions.txt')); 
obsStruc=textscan(obs_id,'%f %f %f %f %f %f');
fclose(obs_id);
obstacles=[ obsStruc{1},obsStruc{2},obsStruc{3},obsStruc{4},obsStruc{5},obsStruc{6}];


%% OBSTACLE & REGIONS

 ObsDir(1,:) = [1 1 1];% A obtacle Vertices direction
 ObsDir(2,:) = [1 1 -1];% B
 ObsDir(3,:) = [1 -1 1];% C
 ObsDir(4,:) = [1 -1 -1];% D
 ObsDir(5,:) = [-1 1 1];% E
 ObsDir(6,:) = [-1 1 -1];% F
 ObsDir(7,:) = [-1 -1 1];% G
 ObsDir(8,:) = [-1 -1 -1];% H

% First line --> Work Space Region
% From 2nd line Obstacle Regions

NumObtacleRegion=max(size(obstacles(:,1)))-1;

for j=1:NumObtacleRegion 
obsCenter(j,:)= obstacles(j+1,1:3);
obsSize(j,:)=   obstacles(j+1,4:6)./2.0;


for i=1:8
    ObsRegion(i,:)= obsCenter(j,:)+obsSize(j,:).*ObsDir(i,:);
end

x=[
    ObsRegion(8,1) ObsRegion(6,1) ObsRegion(2,1) ObsRegion(4,1) ObsRegion(8,1) % bottom
    ObsRegion(7,1) ObsRegion(5,1) ObsRegion(1,1) ObsRegion(3,1) ObsRegion(7,1) % top
    ObsRegion(7,1) ObsRegion(8,1) ObsRegion(4,1) ObsRegion(3,1) ObsRegion(7,1) % left
    ObsRegion(5,1) ObsRegion(6,1) ObsRegion(2,1) ObsRegion(1,1) ObsRegion(5,1) % right
    ];

y=[
    ObsRegion(8,2) ObsRegion(6,2) ObsRegion(2,2) ObsRegion(4,2) ObsRegion(8,2) % bottom
    ObsRegion(7,2) ObsRegion(5,2) ObsRegion(1,2) ObsRegion(3,2) ObsRegion(7,2) % top
    ObsRegion(7,2) ObsRegion(8,2) ObsRegion(4,2) ObsRegion(3,2) ObsRegion(7,2) % left
    ObsRegion(5,2) ObsRegion(6,2) ObsRegion(2,2) ObsRegion(1,2) ObsRegion(5,2) % right
    ];

z=[
    ObsRegion(8,3) ObsRegion(6,3) ObsRegion(2,3) ObsRegion(4,3) ObsRegion(8,3) % bottom
    ObsRegion(7,3) ObsRegion(5,3) ObsRegion(1,3) ObsRegion(3,3) ObsRegion(7,3) % top
    ObsRegion(7,3) ObsRegion(8,3) ObsRegion(4,3) ObsRegion(3,3) ObsRegion(7,3) % left
    ObsRegion(5,3) ObsRegion(6,3) ObsRegion(2,3) ObsRegion(1,3) ObsRegion(5,3) % right
    ];


hold on;
line(x',y',z','color',[0 0 0]);hold on;

end

view(3);
plot3(pathX,pathY,pathZ,'-*');xlabel('x');hold on;
plot3(pathX(1),pathY(1),pathZ(1),'or','MarkerSize',6,'MarkerFaceColor','y','MarkerFaceColor','R');hold on;
ylabel('y');zlabel('z');title('Optimal Trajectory');hold on;

regLimX=[regions(1,1)-regions(1,4)./2.0,regions(1,1)+regions(1,4)./2.0];
regLimY=[regions(1,2)-regions(1,5)./2.0,regions(1,2)+regions(1,5)./2.0];
regLimZ=[regions(1,3)-regions(1,6)./2.0,regions(1,3)+regions(1,6)./2.0];
xlim(regLimX);ylim(regLimY);zlim(regLimZ);grid on;



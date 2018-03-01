clc;clear all;close all;

% Path 
% 4-> 21,22,23 check
path_id= fopen(strcat('./Trajectory.txt')); 
pathStruc=textscan(path_id,'%f %f %f');
fclose(path_id);
path=[ pathStruc{1},pathStruc{2},pathStruc{3}]; 
pathX=path(:,1);
pathY=path(:,2);
pathZ=path(:,3);
% Regions
reg_id= fopen(strcat('./Region.txt')); 
regStruc=textscan(reg_id,'%f %f %f %f %f %f');
fclose(reg_id);
regions=[ regStruc{1},regStruc{2},regStruc{3},regStruc{4},regStruc{5},regStruc{6}];
% Obstacle
obs_id= fopen(strcat('./Obstacles.txt')); 
obsStruc=textscan(obs_id,'%f %f %f %f %f %f');
fclose(obs_id);
obstacles=[ obsStruc{1},obsStruc{2},obsStruc{3},obsStruc{4},obsStruc{5},obsStruc{6}];
% EndEffectorSize
ee_id= fopen(strcat('./EndEffector.txt'));
eeStruc=textscan(ee_id,'%f %f %f');
fclose(ee_id);
endEffector=[ eeStruc{1},eeStruc{2},eeStruc{3}];


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

NumObtacleRegion=max(size(obstacles(:,1)));

for j=1:NumObtacleRegion 
obsCenter(j,:)= obstacles(j,1:3);
obsSize(j,:)=   obstacles(j,4:6)./2.0;


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

NumTrajectoryPoints=max(size(path(:,1)))-1

for j=1:NumTrajectoryPoints 
    trajPointCenter(j,:)= path(j+1,1:3);
    trajPointSize(j,:)=   endEffector(1,1:3)./2.0;
    trajPointLimit(j,:)=   endEffector(1,1:3);
    
    
    for i=1:8
        trajPointRegion(i,:)= trajPointCenter(j,:)+trajPointSize(j,:).*ObsDir(i,:);
        trajLimitRegion(i,:)= trajPointCenter(j,:)+trajPointLimit(j,:).*ObsDir(i,:);
    end
    
    x=[
        trajPointRegion(8,1) trajPointRegion(6,1) trajPointRegion(2,1) trajPointRegion(4,1) trajPointRegion(8,1) % bottom
        trajPointRegion(7,1) trajPointRegion(5,1) trajPointRegion(1,1) trajPointRegion(3,1) trajPointRegion(7,1) % top
        trajPointRegion(7,1) trajPointRegion(8,1) trajPointRegion(4,1) trajPointRegion(3,1) trajPointRegion(7,1) % left
        trajPointRegion(5,1) trajPointRegion(6,1) trajPointRegion(2,1) trajPointRegion(1,1) trajPointRegion(5,1) % right
        ];
    
    y=[
        trajPointRegion(8,2) trajPointRegion(6,2) trajPointRegion(2,2) trajPointRegion(4,2) trajPointRegion(8,2) % bottom
        trajPointRegion(7,2) trajPointRegion(5,2) trajPointRegion(1,2) trajPointRegion(3,2) trajPointRegion(7,2) % top
        trajPointRegion(7,2) trajPointRegion(8,2) trajPointRegion(4,2) trajPointRegion(3,2) trajPointRegion(7,2) % left
        trajPointRegion(5,2) trajPointRegion(6,2) trajPointRegion(2,2) trajPointRegion(1,2) trajPointRegion(5,2) % right
        ];
    
    z=[
        trajPointRegion(8,3) trajPointRegion(6,3) trajPointRegion(2,3) trajPointRegion(4,3) trajPointRegion(8,3) % bottom
        trajPointRegion(7,3) trajPointRegion(5,3) trajPointRegion(1,3) trajPointRegion(3,3) trajPointRegion(7,3) % top
        trajPointRegion(7,3) trajPointRegion(8,3) trajPointRegion(4,3) trajPointRegion(3,3) trajPointRegion(7,3) % left
        trajPointRegion(5,3) trajPointRegion(6,3) trajPointRegion(2,3) trajPointRegion(1,3) trajPointRegion(5,3) % right
        ];
    
    
    
    
    hold on;
    line(x',y',z','color','green')
    

end


view(3);


plot3(pathX,pathY,pathZ,'-*');xlabel('x');hold on;
plot3(pathX(1),pathY(1),pathZ(1),'or','MarkerSize',6,'MarkerFaceColor','y','MarkerFaceColor','R');hold on;
ylabel('y');zlabel('z');title('Optimal Trajectory');hold on;


regLimX=[regions(1,1)-regions(1,4)./2.0,regions(1,1)+regions(1,4)./2.0];
regLimY=[regions(1,2)-regions(1,5)./2.0,regions(1,2)+regions(1,5)./2.0];
regLimZ=[regions(1,3)-regions(1,6)./2.0,regions(1,3)+regions(1,6)./2.0];
xlim(regLimX);ylim(regLimY);zlim(regLimZ);grid on;



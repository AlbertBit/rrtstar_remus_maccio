% clc;clear all;close all;
% 
% path_id= fopen(strcat('/home/nasa/Datalog/Planner/01path.txt')); 
% pathStruc=textscan(path_id,'%u8 %f %f %f');fclose(path_id);
% 
% path=[ pathStruc{2},pathStruc{3},pathStruc{4}];
% plot(path(:,1),path(:,2),'*')
% 




% 
% % clf;
% figure(2);
% format compact 
% h(1) = axes('Position',[0.2 0.2 0.6 0.6]);
% vert = [1 1 -1; 
%         -1 1 -1; 
%         -1 1 1; 
%         1 1 1; 
%         -1 -1 1;
%         1 -1 1; 
%         1 -1 -1;
%         -1 -1 -1];
% 
% fac = [1 2 3 4; 
%        4 3 5 6; 
%        6 7 8 5; 
%        1 2 8 7; 
%        6 7 1 4; 
%        2 3 5 8];
% 
% % I defined a new cube whose length is 1 and centers at the origin.
% vert2 = vert * 0.5;  
% fac2 = fac;
% 
% patch('Faces',fac,'Vertices',vert,'FaceColor','b');  % patch function
% axis([-1, 1, -1, 1, -1, 1]);
% axis equal;
% 
% hold on;
% 
% patch('Faces', fac2, 'Vertices', vert2, 'FaceColor', 'r');
% material metal;
% alpha('color');
% alphamap('rampdown');
% view(3);


close all;

patch([-3.5 -3.5 -6.5 -6.5], [-3.5 -6.5 -6.5 -3.5],[1 0 0]);hold on
patch([3.5 3.5 6.5 6.5], [3.5 6.5 6.5 3.5],[1 0 0]);hold on
plot(VarName1,VarName2);grid on



% function [ T_withTB,T_withoutTB ] = forward_kinematics( q1,q2,q3,q4 )
function [ T_withTB ] = forward_kinematics( q1,q2,q3,q4 )
%UNTITLED4 此处显示有关此函数的摘要
%   此处显示详细说明
% T=forward_kinematics(11,12,13,14)
%-------------------------------
%build scara robot
%scara RRPR (three rotations and one position in order
%DOF=4
%-------------------------------

%set the variables

%use Link command to create link
L1=Link([pi/2 0 40 0 0]);
L2=Link([-pi/2 0 40 0 0]);
L3=Link([0 20 0 0 1 ]);
L4=Link([pi/2 20 0 0 0]);
%set offset for L1
% offset_variables=7;
% L1.offset=offset_variables;
%assemble the robot
SCARA=SerialLink([L1 L2 L3 L4],'name','SCARA1');
%set the limitation,it will use default setting when not given this command
SCARA.qlim=[-pi pi;-pi pi;0 100;-pi pi];
%set the base
 %SCARA.base=transl(0,0,30);
%rotating 
% SCARA.base=SCARA.base*trotx(pi);

%------------------------------------------------------------------

%return the homogeneous trasformation with Toolbox
T_withTB=SCARA.fkine([q1 q2 q3 q4]);

%------------------------------------------------------------------

% %return the homogeneous trasformation without Toolbox
% 
% T_withoutTB=forward_kinematics_withoutTB( q1,q2,q3,q4 );



end



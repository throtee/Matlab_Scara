% function [ T_withTB,T_withoutTB ] = forward_kinematics_withoffsetandbase( q1,q2,q3,q4,base,offset_linknum,offset )
function [ T_withTB,T_withoutTB ] = forward_kinematics_withoffsetandbase( q1,q2,q3,q4,base )
%UNTITLED4 此处显示有关此函数的摘要
%   此处显示详细说明

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
% offset_variables=offset;
% L1.offset=offset_variables;
%assemble the robot
SCARA=SerialLink([L1 L2 L3 L4],'name','SCARA1');
%set the limitation,it will use default setting when not given this command
SCARA.qlim=[-pi pi;-pi pi;0 100;-pi pi];
%set the base
 SCARA.base=transl(0,0,base);
%rotating 
% SCARA.base=SCARA.base*trotx(pi);

%------------------------------------------------------------------

%return the homogeneous trasformation with Toolbox
T_withTB=SCARA.fkine([q1 q2 q3 q4]);

%------------------------------------------------------------------

%return the homogeneous trasformation without Toolbox

% T_withoutTB=forward_kinematics_withoutTB( q1,q2,q3,q4,base,offset_linknum,offset );
T_withoutTB=forward_kinematics_withoutTB( q1,q2,q3,q4,base);


end

% function T_withoutTB=forward_kinematics_withoutTB( q1,q2,q3,q4,base,offset_linknum,offset )
function T_withoutTB=forward_kinematics_withoutTB( q1,q2,q3,q4,base )

%set the variables
L1=[q1 0 40 0 0];
L2=[q2 0 40 0 0];
L3=[0 q3 0 0 1 ];
L4=[q4 20 0 0 0];

L_SCARA=[L1;L2;L3;L4];

% if offset_linknum==1
%     L_SCARA(1,1)=L_SCARA(1,1)+offset;
% else
%     if offset_linknum==2
%         L_SCARA(2,1)=L_SCARA(2,1)+offset;
%     else
%         if offset_linknum==3
%             L_SCARA(3,2)=L_SCARA(3,2)+offset;
%         else
%             if offset_linknum==4
%                 L_SCARA(4,1)=L_SCARA(4,1)+offset;
% end
% 

T_withoutTB=eye(4);

for i=1:4
    theta_tem=L_SCARA(i,1);
    d_tem=L_SCARA(i,2);
    a_tem=L_SCARA(i,3);
    alpha_tem=L_SCARA(i,4);
    
    A_tem=[cos(theta_tem),-sin(theta_tem)*cos(alpha_tem),sin(theta_tem)*sin(alpha_tem),a_tem*cos(theta_tem);
        sin(theta_tem),cos(theta_tem)*cos(alpha_tem),-cos(theta_tem)*sin(alpha_tem),a_tem*sin(theta_tem);
        0,sin(alpha_tem),cos(alpha_tem),d_tem;
        0,0,0,1];
    
     %sum_A(:,i)=A_tem;
 
    T_withoutTB=T_withoutTB*A_tem;
end

T_withoutTB(3,4)=T_withoutTB(3,4)+base;

end

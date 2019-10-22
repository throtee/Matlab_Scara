function [ q ]= inverse_kinematics( T,I11,I22,I33,I44 )
L1=Link([pi/2 0 40 0 0]);
L2=Link([-pi/2 0 40 0 0]);
L3=Link([0 20 0 0 1 ]);
L4=Link([pi/2 20 0 0 0]);
SCARA=SerialLink([L1 L2 L3 L4],'name','SCARA1');
SCARA.qlim=[-pi pi;-pi pi;0 100;-pi pi];
q=SCARA.ikine(T,[I11 I22 I33 I44],[1 1 1 1 0 0]);
end
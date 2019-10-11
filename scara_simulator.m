
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
offset_variables=7;
L1.offset=offset_variables;
%assemble the robot
SCARA=SerialLink([L1 L2 L3 L4],'name','SCARA1');
%set the limitation,it will use default setting when not given this command
SCARA.qlim=[-pi pi;-pi pi;0 100;-pi pi];
%set the base
SCARA.base=transl(0,0,30);
%rotating 
% SCARA.base=SCARA.base*trotx(pi);

%------------------------------------------------------------------

%plot

%workplace is needed for setting the qlim(possibly
%SCARA.plot([0 0 0 0],'workspace',[-120 120 -120 120 -120 120])
%it is used when given the sepcific 'qlim'
%SCARA.teach

%---------------------------------------------------------------------

%kinematics

%forward kinematics,q1-4 for set the movement of 4 link,return the
%homogeneous transformation matrix
q1=0;
q2=0;
q3=0;
q4=0;
T=SCARA.fkine([q1 q2 q3 q4]);

%reverse kinematics-closed-form solution
%ikine6s is for 6DOF
%left-right hand 'l','r'
%elbow up-down 'u','d'
%wrist flip-not 'f','n'
%Because joint axes coincide, the degree of freedom is reduced which
%creates singularity motion.The data of two joint may be random number, but
%overall the sum of the two joint data is constant. It can be understood as
%two joint connected as one.
%exp: qz=p560.ikine6s(T,'rd');

%for less than 6DOF a mask matrix must be apllied
T_cfs=transl(40,65,130);
q_cfs=SCARA.ikine(T_cfs,[0 0 0 0],[1 1 1 0 0 1]);
%it will only consider x,y,z axis and alpha,other coordinates value is
%wrong
T_cfs_new=SCARA.fkine(q_cfs);


%------------------------

%set the trajectory

%get the position and joint coordinates
T1=transl(-100,100,50);
T2=transl(-46,64,78);
q1=SCARA.ikine(T1,[0 0 0 0],[1 1 1 0 0 1]);
q2=SCARA.ikine(T2,[0 0 0 0],[1 1 1 0 0 1]);
%set the time pulse
t=(0:0.05:2)';

q=mtraj(@tpoly,q1,q2,t);
SCARA.plot(q,'workspace',[-120 120 -120 120 -120 120]);
plot(t,q(:,2));








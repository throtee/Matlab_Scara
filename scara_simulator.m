
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

%------------------------------------------------------------------

%plot

%workplace is needed for setting the qlim(possibly
SCARA.plot([0 0 0 0],'workspace',[-120 120 -120 120 -120 120])
%it is used when given the sepcific 'qlim'
SCARA.teach

%---------------------------------------------------------------------

%

%forward kinematics,q1-4 for set the movement of 4 link,return the Position
q1=0;
q2=0;
q3=0;
q4=0;
SCARA.fkine([q1 q2 q3 q4])


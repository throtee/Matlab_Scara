function T_withoutTB=forward_kinematics_withoutTB( q1,q2,q3,q4 )

%set the variables
L1=[q1 0 40 0 0];
L2=[q2 0 40 0 0];
L3=[0 q3 0 0 1 ];
L4=[q4 20 0 0 0];

L_SCARA=[L1;L2;L3;L4];

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


end
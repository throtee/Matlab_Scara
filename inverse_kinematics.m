function [q1,q2,q3,q4]=inverse_kinematics(T)
% syms q1_J q2_J q3_J q4_J
syms q1_C q2_C q3_C q4_C
syms q1 q2 q3 q4

% q1_C=0;q2_C=0;q3_C=0;q4_C=0;

q_sum=[q1,q2,q3,q4]';
q_sum_C=[q1_C,q2_C,q3_C,q4_C]';

% p=T*p_0;
% 
% T_sum=T1*T2*T3*T4;

T_IK=T_maker(q1,q2,q3,q4);
T_sum_C=T_maker(q1_C,q2_C,q3_C,q4_C);


% T_result=[T_TK(11),T_TK(21),T_TK(12),T_TK(22),T_TK(32),T_TK(13),T_TK(23),T_TK(33),T_TK(14),T_TK(24),T_TK(34)]';

Jacobian_T=jacobian([T_IK(11),T_IK(21),T_IK(12),T_IK(22),T_IK(32),T_IK(13),T_IK(23),T_IK(33),T_IK(14),T_IK(24),T_IK(34)],[q1,q2,q3,q4]);

T_result=T_sum_C+Jacobian_T*(q_sum-q_sum_C);





end


function T=T_maker(q1,q2,q3,q4)

%set the variables

%use Link command to create link
L1=Link([q1 0 40 0 0]);
L2=Link([q2 0 40 0 0]);
L3=Link([0 q3 0 0 1 ]);
L4=Link([q4 20 0 0 0]);
L_SCARA=[L1;L2;L3;L4];

T=eye(4);

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
 
    T=T*A_tem;
end


end

% function [q1,q2,q3,q4]=T_converge(T)
% end
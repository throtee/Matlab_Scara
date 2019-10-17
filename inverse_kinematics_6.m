function [q1,q2,q3,q4]=inverse_kinematics_6(T)
%the problem can be describe as input the homogeneous transformation matrix of SCARA and get
%the D-H variables of 'goal' position

%define as 'current' variables
syms q1_C q2_C q3_C q4_C
%define as 'goal' variables
% syms q1 q2 q3 q4
syms I1 I2 I3 I4
I1=0; I2=0; I3=0; I4=0;
% q1_C=0;q2_C=0;q3_C=0;q4_C=0;
% q_sum=[q1,q2,q3,q4]';
% q_sum_C=[q1_C,q2_C,q3_C,q4_C]';
% T_IK=T_maker(q1,q2,q3,q4);

%transform the 'current' matrix
% T_sum_C=T_maker(q1_C,q2_C,q3_C,q4_C);

%----------------------------------------------------------------
%set the variables
%the function is used for creating homogeneous transformation matrix
L1=[q1_C 0 40 0 0];
L2=[q2_C 0 40 0 0];
L3=[0 q3_C 0 0 1 ];
L4=[q4_C 20 0 0 0];
L_SCARA=[L1;L2;L3;L4];

T_sum_C=eye(4);

%create matrix
for i=1:4
    theta_tem=L_SCARA(i,1);
    d_tem=L_SCARA(i,2);
    a_tem=L_SCARA(i,3);
    alpha_tem=L_SCARA(i,4);
    
    A_tem=[cos(theta_tem),-sin(theta_tem)*cos(alpha_tem),sin(theta_tem)*sin(alpha_tem),a_tem*cos(theta_tem);
        sin(theta_tem),cos(theta_tem)*cos(alpha_tem),-cos(theta_tem)*sin(alpha_tem),a_tem*sin(theta_tem);
        0,sin(alpha_tem),cos(alpha_tem),d_tem;
        0,0,0,1];
    
 
    T_sum_C=T_sum_C*A_tem;
end

%--------------------------------------------------------------------------------------
T_sum_C_T=[T_sum_C(1,1),T_sum_C(2,1),T_sum_C(1,2),T_sum_C(2,2),T_sum_C(3,2),T_sum_C(1,3),T_sum_C(2,3),T_sum_C(3,3),T_sum_C(1,4),T_sum_C(2,4),T_sum_C(3,4)]';
Jacobian_T=jacobian([T_sum_C(1,1),T_sum_C(2,1),T_sum_C(1,2),T_sum_C(2,2),T_sum_C(3,2),T_sum_C(1,3),T_sum_C(2,3),T_sum_C(3,3),T_sum_C(1,4),T_sum_C(2,4),T_sum_C(3,4)],[q1_C,q2_C,q3_C,q4_C]);
% q1_C=I1;q2_C=I2;q3_C=I3;q4_C=I4;
% Jacobian_T_C=subs(Jacobian_T);
T_T=[T(1,1),T(2,1),T(1,2),T(2,2),T(3,2),T(1,3),T(2,3),T(3,3),T(1,4),T(2,4),T(3,4)]';

% T_result=T_sum_C_T+Jacobian_T*(q_sum-q_sum_C);

%-----------------------------------------------------------------
%IT SHOULD BUILD IN AS A ROTATION, IT IS THE RIGHT WAY MATHMATICALLY 
q1_C=I1;q2_C=I2;q3_C=I3;q4_C=I4;

Jacobian_Inverted=double(subs(Jacobian_T));
% Jacobian_Inverted=subs(pinv(Jacobian_T)');
Jacobian_Inverted_C=invert_SVD(Jacobian_Inverted);
T_sum_C_T_T=subs(T_sum_C_T);

% q_delta=Jacobian_Inverted_C*(T_T-T_sum_C_T_T);
% 
% while q_delta~=0
% q_delta=Jacobian_Inverted_C*(T_T-T_sum_C_T_T);
% q1_C=q1_C+q_delta(1); q2_C=q2_C+q_delta(2); q3_C=q3_C+q_delta(3); q3_C=q3_C+q_delta(3);
% % Jacobian_Inverted=subs(pinv(Jacobian_T)');
% % Jacobian_Inverted_C=subs(Jacobian_Inverted);
% 
% Jacobian_Inverted=double(subs(Jacobian_T));
% Jacobian_Inverted_C=invert_SVD(Jacobian_Inverted);
% T_sum_C_T_T=subs(T_sum_C_T);
% end

for i=1:5
q_delta=Jacobian_Inverted_C*(T_T-T_sum_C_T_T);
q1_C=q1_C+q_delta(1); q2_C=q2_C+q_delta(2); q3_C=q3_C+q_delta(3); q3_C=q3_C+q_delta(3);
% Jacobian_Inverted=subs(pinv(Jacobian_T)');
% Jacobian_Inverted_C=subs(Jacobian_Inverted);

Jacobian_Inverted=double(subs(Jacobian_T));
Jacobian_Inverted_C=invert_SVD(Jacobian_Inverted);
T_sum_C_T_T=subs(T_sum_C_T);
end

% q1_C=0;q2_C=0;q3_C=0;q4_C=0;

% if abs(q_delta(1)/q_delta())
    
%------------------------------------------------------------------

q1=double(q1_C);q2=double(q2_C);q3=double(q3_C);q4=double(q4_C);

end

function T=invert_SVD(A)
[U,S,V] = svd(A);
%get S_plus-----------------------------------------------------------
S_plus=S;
S_1=S~=0;
S_2=1./S(S_1);
S_plus(S_1)=S_2;
%--------------------------------------------------------------------
T=V'*S_plus'*U;
end





% function [q1,q2,q3,q4]=T_converge(T)
% end
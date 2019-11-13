%-------------------------------------------------------------------
%Made by: Enze Chen 3072167875@qq.com
%inspired by:Niels Joubert http://www-inst.cs.berkeley.edu/~cs184/fa09/resources/ik_soln.pdf
%-----------------------------------------------------------------
%Description: the function is used to input the homogeneous transformation
%matrix of SCARA and get the D-H variables of 'goal' position
%-----------------------------------------------------------------
%Variables:
%q1-4:The variables in the D-H parameter match to the transformation matrix
%变换矩阵对应的D-H参数中的变量
%T:transformation matrix of the goal position
%目标点变换矩阵
%I11-44:The variables in the D-H parameter of the initial position
%D-H参数中的变量的初始位置
%q1_C q2_C q3_C q4_C: D-H variables of the current position
%D-H参数中的变量的当前位置
%T_sum:Transformation matrix constructed by variables in the dh parameter
%以dh参数中的变量表示的变换矩阵
%Jacobian_T：Jacobian transformation matrix
%雅可比变换矩阵
%T_sum1：a matrix containing the dh parameter variables in the transformation matrix
%包含变换矩阵里dh参数变量的矩阵
%Jacobian_Inverted_C:Pseudo-inverse Jacobian matrix after assigning a variable
%给变量赋值后的伪逆雅可比矩阵
%T_sum1_C:'T_sum1'after assigning a variable
%给变量赋值后的'T_sum1'
%q_delta：The difference between the goal coordinates and the current coordinates
%终点坐标和当前坐标的差值



function [q1,q2,q3,q4]=inverse_kinematics_withoutTB(T,I11,I22,I33,I44)

%set the variables

%define as 'current' D-H variables
syms q1_C q2_C q3_C q4_C
%define as storage of 'current' D-H variables value, default is 0
syms I1 I2 I3 I4
switch nargin
    case 1
        I1=10; I2=10; I3=10; I4=0;
    otherwise
        I1=I11; I2=I22; I3=I33; I4=I44;
end
%----------------------------------------------------------------

%construct the SCARA matrix

%create link
L1=[q1_C 0 425 0];
L2=[q2_C 0 275 0];
L3=[0 q3_C 0 0 ];
L4=[q4_C 160 0 0];
L_SCARA=[L1;L2;L3;L4];

%generate the default homogeneous transformation matrix
T_sum=eye(4);

%create the homogeneous transformation matrix
for i=1:4
    theta_tem=L_SCARA(i,1);
    d_tem=L_SCARA(i,2);
    a_tem=L_SCARA(i,3);
    alpha_tem=L_SCARA(i,4);
    
    A_tem=[cos(theta_tem),-sin(theta_tem)*cos(alpha_tem),sin(theta_tem)*sin(alpha_tem),a_tem*cos(theta_tem);
        sin(theta_tem),cos(theta_tem)*cos(alpha_tem),-cos(theta_tem)*sin(alpha_tem),a_tem*sin(theta_tem);
        0,sin(alpha_tem),cos(alpha_tem),d_tem;
        0,0,0,1];
    
    T_sum=T_sum*A_tem;
end

%--------------------------------------------------------------------------------------

%transform the jacobian matrix,homogeneous transformation
%matrix, homogeneous transformation matrix of the goal position to fit the
%calculation format of q_delta
Jacobian_T=jacobian([T_sum(1,1),T_sum(2,1),T_sum(1,2),T_sum(2,2), ...
    T_sum(3,2),T_sum(1,3),T_sum(2,3),T_sum(3,3),T_sum(1,4), ...
    T_sum(2,4),T_sum(3,4)],[q1_C,q2_C,q3_C,q4_C]);
T_sum1=[T_sum(1,1),T_sum(2,1),T_sum(1,2),T_sum(2,2),T_sum(3,2), ...
    T_sum(1,3),T_sum(2,3),T_sum(3,3),T_sum(1,4),T_sum(2,4),T_sum(3,4)]';
T1=[T(1,1),T(2,1),T(1,2),T(2,2),T(3,2),T(1,3),T(2,3),T(3,3),T(1,4),T(2,4),T(3,4)]';

%-----------------------------------------------------------------

%IT SHOULD BUILD IN AS A ROTATION, IT IS THE RIGHT WAY MATHMATICALLY 
%solve and get the D-H variables of 'goal' position

%give the D-H variables the storage value
q1_C=I1;q2_C=I2;q3_C=I3;q4_C=I4;

%transform the value to the jacobian matrix and inverse it
%note: SVD is not used, it could be used for learning purposes but my
%original 'invert_SVD' function has some error unfixed, it is shown below.
Jacobian_T_C=double(subs(Jacobian_T));
Jacobian_Inverted_C=invert_SVD(Jacobian_T_C);
T_sum1_C=subs(T_sum1);
q_delta=Jacobian_Inverted_C*(T1-T_sum1_C);

% if i=1:100
% end

%loop for iteration to reduce the distance between goal position and
%current position, you can get a more accurate value by set the loop for a
%fixed amount of times
%it may not converge, when it happens half the 'q_delta' and try again
while q_delta~=0
q_delta=Jacobian_Inverted_C*(T1-T_sum1_C);
q1_C=double(q1_C+q_delta(1)); q2_C=double(q2_C+q_delta(2)); q3_C=double(q3_C+q_delta(3)); q4_C=double(q4_C+q_delta(4));

Jacobian_T_C=double(subs(Jacobian_T));
Jacobian_Inverted_C=invert_SVD(Jacobian_T_C);
T_sum1_C=subs(T_sum1);
end

    
%------------------------------------------------------------------

q1=double(q1_C);q2=double(q2_C);q3=double(q3_C);q4=double(q4_C);
q1=wrapToPi(q1); q2=wrapToPi(q2); q4=wrapToPi(q4);
end


% function T=invert_SVD(A)
% [U,S,V] = svd(A);
% %get S_plus-----------------------------------------------------------
% S_plus=S;
% S_1=S~=0;
% S_2=1./S(S_1);
% S_plus(S_1)=S_2;
% %--------------------------------------------------------------------
% T=V'*S_plus'*U;
% end
function T=invert_SVD(A)
T=pinv(A);
end


% function [q1,q2,q3,q4]=T_converge(T)
% end
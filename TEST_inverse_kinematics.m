function [ q1_withoutTB,q2_withoutTB,q3_withoutTB,q4_withoutTB,q1_withTB,q2_withTB,q3_withTB,q4_withTB ] = TEST_inverse_kinematics( T,I11,I22,I33,I44 )
switch nargin
    case 1
        I1=0; I2=0; I3=0; I4=0;
    otherwise
        I1=I11; I2=I22; I3=I33; I4=I44;
end
q =inverse_kinematics(T,I1,I2,I3,I4);
[q1_withoutTB,q2_withoutTB,q3_withoutTB,q4_withoutTB]= inverse_kinematics_withoutTB( T,I1,I2,I3,I4 );
q1_withTB=q(1); q2_withTB=q(2); q3_withTB=q(3); q4_withTB=q(4); 
end


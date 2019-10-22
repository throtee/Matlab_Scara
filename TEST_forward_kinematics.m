function  [ T_withTB,T_withoutTB ]=TEST_forward_kinematics(q1,q2,q3,q4)
T_withoutTB=forward_kinematics_withoutTB( q1,q2,q3,q4 );
T_withTB = forward_kinematics( q1,q2,q3,q4 );
end


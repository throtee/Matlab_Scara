%tranform (transl)
function [t]=translE(q1,q2,q3)
t1=eye(4);
t1(1,4)=q1;
t1(2,4)=q2;
t1(3,4)=q3;
t=t1;
end
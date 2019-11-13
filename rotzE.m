function [t]=rotzE(q)
tz=eye(3);
tz(1,1)=cos(q);
tz(1,2)=-sin(q);
tz(2,1)=sin(q);
tz(2,2)=cos(q);
t=tz;
end
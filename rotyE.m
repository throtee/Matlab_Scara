function [t]=rotyE(q)
ty=eye(3);
ty(1,1)=cos(q);
ty(3,1)=-sin(q);
ty(1,3)=sin(q);
ty(3,3)=cos(q);
t=ty;
end
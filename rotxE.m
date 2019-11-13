function [t]=rotxE(q)
tx=eye(3);
tx(2,2)=cos(q);
tx(2,3)=-sin(q);
tx(3,2)=sin(q);
tx(3,3)=cos(q);
t=tx;
end
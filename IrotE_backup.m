function [ux,vy,wz]=IrotE(t)
%default z,y,x
syms v u w
t2=zeros(3,3);
Ind=0;
%Rotation matrix of X,Y,Z
tx=sym(eye(3));
tx(2,2)=cos(u);
tx(2,3)=-sin(u);
tx(3,2)=sin(u);
tx(3,3)=cos(u);
ty=sym(eye(3));
ty(1,1)=cos(v);
ty(3,1)=-sin(v);
ty(1,3)=sin(v);
ty(3,3)=cos(v);
tz=sym(eye(3));
tz(1,1)=cos(w);
tz(1,2)=-sin(w);
tz(2,1)=sin(w);
tz(2,2)=cos(w);
% switch nargin
% %     case 4
% %         
%     case 1
t1=tz*ty*tx;
%           [qx,qy,qz] = vpasolve([t1(1,1)==t(1,1),t1(1,2)==t(1,2),t1(1,3)==t(1,3), ...
%               t1(2,1)==t(2,1),t1(2,2)==t(2,2),t1(2,3)==t(2,3), ...
%               t1(3,1)==t(3,1),t1(3,2)==t(3,2),t1(3,3)==t(3,3)], [q1,q2,q3]);
% [ux,vy,wz] = vpasolve([t1(1,1)==t(1,1),t1(1,2)==t(1,2),t1(1,3)==t(1,3)], [u,v,w]);
while ~isequal(t2,t)
    for i=1:3
        Ind=Ind+1;
        tool1=round(rand*2)+1;
        Indmat(Ind,1)=tool1;
        tool2=round(rand*2)+1;
        Indmat(Ind,2)=tool2;
        eval(['a',num2str(i),'=','[tool1,tool2]',';']);
    end
    if ((Ind-1)/3)>1
        for j=1:((Ind-1)/3)
            isequal(Indmat( (((Ind-1)/3)-j) , : ),
        end
    end
    if ~isequal(a1,a2)&&~isequal(a2,a3)&&~isequal(a2,a3)
        [ux,vy,wz] = vpasolve([t1(a1(1),a1(2))==t(a1(1),a1(2)), ...
            t1(a2(1),a2(2))==t(a2(1),a2(2)), ...
            t1(a3(1),a3(2))==t(a3(1),a3(2))], [u,v,w]);
        ux=wrapToPi(double(ux)); vy=wrapToPi(double(vy)); wz=wrapToPi(double(wz));
        if ~isempty(ux)
%             &&~isempty(vy)&&~isempty(wz)
            t2=rotzE(wz)*rotyE(vy)*rotxE(ux);
            %         t2=t3;
            %         t3=zeros(3,3);
        end
        
        %     otherwise
        %         fprintf('error:invalid input');
        %         return
        % end
        % ux=wrapToPi(double(ux)); vy=wrapToPi(double(vy)); wz=wrapToPi(double(wz));
    end
end
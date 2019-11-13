function [ux,vy,wz]=IrotE(t,q1,q2,q3)
%default z,y,x
syms v u w
% syms w1 w2 w3 u1 u2 u3 v1 v2 v3
b(:,:,1)=sym(eye(3));
b(:,:,2)=sym(eye(3));
b(:,:,3)=sym(eye(3));
t2=zeros(3,3);
q=[q1,q2,q3];
% syms w1 w2 w3 u1 u2 u3 v1 v2 v3
% ut=[u1,u2,u3];
% vt=[v1,v2,v3];
% wt=[w1,w2,w3];
% Axi=[ut;vt;wt];
j=0;

if q1=='x'
    q11='ux';
end
if q1=='y'
    q11='vy';
end
if q1=='z'
    q11='wz';
end
if q2=='x'
    q22='ux';
end
if q2=='y'
    q22='vy';
end
if q2=='z'
    q22='wz';
end
if q3=='x'
    q33='ux';
end
if q3=='y'
    q33='vy';
end
if q3=='z'
    q33='wz';
end

%Rotation matrix of X,Y,Z
for i=1:3
    if q(i)=='x'
        %         b(i)=sym(eye(3));
        j=j+1;
        switch j
            case 1
%                 u=Axi(1,i);
                b(2,2,i)=cos(u);
                b(2,3,i)=-sin(u);
                b(3,2,i)=sin(u);
                b(3,3,i)=cos(u);
            case 2
%                 v=Axi(1,i);
                b(2,2,i)=cos(v);
                b(2,3,i)=-sin(v);
                b(3,2,i)=sin(v);
                b(3,3,i)=cos(v);
            case 3
%                 w=Axi(1,i);
                b(2,2,i)=cos(w);
                b(2,3,i)=-sin(w);
                b(3,2,i)=sin(w);
                b(3,3,i)=cos(w);
        end
    end
    if q(i)=='y'
        %         b(i)=sym(eye(3));
        j=j+1;
        switch j
            case 1
%                 u=Axi(2,i);
                b(1,1,i)=cos(u);
                b(3,1,i)=-sin(u);
                b(1,3,i)=sin(u);
                b(3,3,i)=cos(u);
            case 2
%                 v=Axi(2,i);
                b(1,1,i)=cos(v);
                b(3,1,i)=-sin(v);
                b(1,3,i)=sin(v);
                b(3,3,i)=cos(v);
            case 3
%                 w=Axi(2,i);
                b(1,1,i)=cos(w);
                b(3,1,i)=-sin(w);
                b(1,3,i)=sin(w);
                b(3,3,i)=cos(w);
        end
    end
    if q(i)=='z'
        %         b(i)=sym(eye(3));
        j=j+1;
        switch j
            case 1
%                 u=Axi(2,i);
                b(1,1,i)=cos(u);
                b(1,2,i)=-sin(u);
                b(2,1,i)=sin(u);
                b(2,2,i)=cos(u);
            case 2
%                 v=Axi(2,i);
                b(1,1,i)=cos(v);
                b(1,2,i)=-sin(v);
                b(2,1,i)=sin(v);
                b(2,2,i)=cos(v);
            case 3
%                 w=Axi(2,i);
                b(1,1,i)=cos(w);
                b(1,2,i)=-sin(w);
                b(2,1,i)=sin(w);
                b(2,2,i)=cos(w);
        end
    end
end
% tx=sym(eye(3));
% tx(2,2)=cos(u);
% tx(2,3)=-sin(u);
% tx(3,2)=sin(u);
% tx(3,3)=cos(u);
% ty=sym(eye(3));
% ty(1,1)=cos(v);
% ty(3,1)=-sin(v);
% ty(1,3)=sin(v);
% ty(3,3)=cos(v);
% tz=sym(eye(3));
% tz(1,1)=cos(w);
% tz(1,2)=-sin(w);
% tz(2,1)=sin(w);
% tz(2,2)=cos(w);
switch nargin
    case 1
        
    case 4
%         b1=eval(['t',q1]);
%         b2=eval(['t',q2]);
%         b3=eval(['t',q3]);
        t1=b(:,:,1)*b(:,:,2)*b(:,:,3);
        t1
%         t1
%         t1=tz*ty*tx;
        %           [qx,qy,qz] = vpasolve([t1(1,1)==t(1,1),t1(1,2)==t(1,2),t1(1,3)==t(1,3), ...
        %               t1(2,1)==t(2,1),t1(2,2)==t(2,2),t1(2,3)==t(2,3), ...
        %               t1(3,1)==t(3,1),t1(3,2)==t(3,2),t1(3,3)==t(3,3)], [q1,q2,q3]);
        % [ux,vy,wz] = vpasolve([t1(1,1)==t(1,1),t1(1,2)==t(1,2),t1(1,3)==t(1,3)], [u,v,w]);
        while ~isequal(t2,t)
            for i=1:3
                tool1=round(rand*2)+1;
                tool2=round(rand*2)+1;
                eval(['a',num2str(i),'=','[tool1,tool2]',';']);
                %         a1
            end
            if ~isequal(a1,a2)&&~isequal(a2,a3)&&~isequal(a1,a3)
                a1
                a2
                a3
                [ux,vy,wz] = vpasolve([t1(a1(1),a1(2))==t(a1(1),a1(2)), ...
                    t1(a2(1),a2(2))==t(a2(1),a2(2)), ...
                    t1(a3(1),a3(2))==t(a3(1),a3(2))], [u,v,w]);
                ux=wrapToPi(double(ux)); vy=wrapToPi(double(vy)); wz=wrapToPi(double(wz));
                %                 ux
                %                 vy
                %                 wz
                if ~isempty(ux)
                    %                     ||~isempty(vy)||~isempty(wz)
                    %             &&~isempty(vy)&&~isempty(wz)
                    %                     t2=rotzE(wz)*rotyE(vy)*rotxE(ux);
                    %                    t2=eval(['rot',q1,'E(',q11,')'])*eval(['rot',q2,'E(',q22,')'])*eval(['rot',q3,'E(',q33,')']);
                    t2=eval(['rot',q1,'E(ux)'])*eval(['rot',q2,'E(vy)'])*eval(['rot',q3,'E(wz)']);
                    %                     t2
                end
                %     otherwise
                %         fprintf('error:invalid input');
                %         return
            end
            % ux=wrapToPi(double(ux)); vy=wrapToPi(double(vy)); wz=wrapToPi(double(wz));
        end
end
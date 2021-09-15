function [P_out]=AStar(map,Spoint,Epoint)

% Spoint = [3 3];	  %起始点坐标
% Epoint = [290 220];	%目标点坐标
Matrix=map;
[m,n]=size(map);
m=m-2;n=n-2;
% n=500;m=500;

%%寻路
Matrix(Spoint(1),Spoint(2))=0;
Matrix(Epoint(1),Epoint(2))=inf;
G=Matrix;
F=Matrix;
openlist=Matrix;
closelist=Matrix;
parentx=Matrix;
parenty=Matrix;
openlist(Spoint(1),Spoint(2)) =0;

% figure,
% for i = 1:m+2
%     for j = 1:n+2
%         k = Matrix(i,j);
%         if(k == -inf)
%             h3 = plot(i,j,'k.');
%         end
%         hold on
%     end
% end
% axis([0 m+3 0 n+3]);
% plot(Epoint(1),Epoint(2),'b+');
% plot(Spoint(1),Spoint(2),'b+');

while(1)
    num=inf;
    for p=1:m+2
        for q=1:n+2
            if(openlist(p,q)==0&&closelist(p,q)~=1)
                Outpoint=[p,q];
                if(F(p,q)>=0&&num>F(p,q))
                    num=F(p,q);
                    Nextpoint=[p,q];
                end
            end
        end
    end
    closelist(Nextpoint(1),Nextpoint(2))=1;
    for i = 1:3
        for j = 1:3
            k = G(Nextpoint(1)-2+i,Nextpoint(2)-2+j);
            if(i==2&&j==2||closelist(Nextpoint(1)-2+i,Nextpoint(2)-2+j)==1)
                continue;
            elseif (k == -inf)
                G(Nextpoint(1)-2+i,Nextpoint(2)-2+j) = G(Nextpoint(1)-2+i,Nextpoint(2)-2+j);
                closelist(Nextpoint(1)-2+i,Nextpoint(2)-2+j)=1;
            elseif (k == inf)
                distance=((i-2)^2+(j-2)^2)^0.5;
                G(Nextpoint(1)-2+i,Nextpoint(2)-2+j)=G(Nextpoint(1),Nextpoint(2))+distance;
                openlist(Nextpoint(1)-2+i,Nextpoint(2)-2+j)=0;
                % H=((Nextpoint(1)-2+i-Epoint(1))^2+(Nextpoint(2)-2+j-Epoint(2))^2)^0.5;%欧几里德距离启发函数
                H_diagonal=min(abs(Nextpoint(1)-2+i-Epoint(1)),abs(Nextpoint(2)-2+j-Epoint(2)));%比较复杂的对角线启发函数
                H_straight=abs(Nextpoint(1)-2+i-Epoint(1))+abs(Nextpoint(2)-2+j-Epoint(2));
                H=sqrt(2)*H_diagonal+(H_straight-2*H_diagonal);
                % H=max(abs(Nextpoint(1)-2+i-Epoint(1)),abs(Nextpoint(2)-2+j-Epoint(2)));%比较简单的对角线函数
                F(Nextpoint(1)-2+i,Nextpoint(2)-2+j)=G(Nextpoint(1)-2+i,Nextpoint(2)-2+j)+H;
                parentx(Nextpoint(1)-2+i,Nextpoint(2)-2+j)=Nextpoint(1);
                parenty(Nextpoint(1)-2+i,Nextpoint(2)-2+j)=Nextpoint(2);
            else distance=((i-2)^2+(j-2)^2)^0.5;
                if(k>(distance+G(Nextpoint(1),Nextpoint(2))))
                    k=distance+G(Nextpoint(1),Nextpoint(2));
                    % H=((Nextpoint(1)-2+i-Epoint(1))^2+(Nextpoint(2)-2+j-Epoint(2))^2)^0.5;  %欧几里德距离启发函数
                    H_diagonal=min(abs(Nextpoint(1)-2+i-Epoint(1)),abs(Nextpoint(2)-2+j-Epoint(2)));%比较复杂的对角线启发函数
                    H_straight=abs(Nextpoint(1)-2+i-Epoint(1))+abs(Nextpoint(2)-2+j-Epoint(2));
                    H=sqrt(2)*10*H_diagonal+10*(H_straight-2*H_diagonal);
                    % H=max(abs(Nextpoint(1)-2+i-Epoint(1)),abs(Nextpoint(2)-2+j-Epoint(2)));%比较简单的对角线函数
                    F(Nextpoint(1)-2+i,Nextpoint(2)-2+j)=k+H;
                    parentx(Nextpoint(1)-2+i,Nextpoint(2)-2+j)=Nextpoint(1);
                    parenty(Nextpoint(1)-2+i,Nextpoint(2)-2+j)=Nextpoint(2);
                end
            end
            if(((Nextpoint(1)-2+i)==Epoint(1)&&(Nextpoint(2)-2+j)==Epoint(2))|num==inf)
                parentx(Epoint(1),Epoint(2))=Nextpoint(1);
                parenty(Epoint(1),Epoint(2))=Nextpoint(2);
                break;
            end
        end
        if(((Nextpoint(1)-2+i)==Epoint(1)&&(Nextpoint(2)-2+j)==Epoint(2))|num==inf)
            parentx(Epoint(1),Epoint(2))=Nextpoint(1);
            parenty(Epoint(1),Epoint(2))=Nextpoint(2);
            break;
        end
    end
    if(((Nextpoint(1)-2+i)==Epoint(1)&&(Nextpoint(2)-2+j)==Epoint(2))|num==inf)
        parentx(Epoint(1),Epoint(2))=Nextpoint(1);
        parenty(Epoint(1),Epoint(2))=Nextpoint(2);
        break;
    end
end
P=[];
s=1;
while(1)
    if(num==inf)
        break;
    end
    %subplot(2,2,1);
%     h4 = plot(Epoint(1),Epoint(2),'b+');
    P(s,:)=Epoint;
    s=s+1;
    % pause(1);
    xx=Epoint(1);
    Epoint(1)=parentx(Epoint(1),Epoint(2));
    Epoint(2)=parenty(xx,Epoint(2));
    if(parentx(Epoint(1),Epoint(2))==Spoint(1)&&parenty(Epoint(1),Epoint(2))==Spoint(2))
        %subplot(2,2,1);
%         plot(Epoint(1),Epoint(2),'b+');
        P(s,:)=Epoint;
        break;
    end
end
P(s+1,:)=Spoint;
P_out=P;
end
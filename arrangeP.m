function [ts_out,route_out,ts_k_out]=arrangeP(fx1,fx2,fy1,fy2,ft)
% 输入曲线，输出半径节点以及相应的时间规划
% load('arrangeP.mat');
global r_max

rot=[0 -1;
    1 0];
ff1=[fx1;
    fy1];
ff2=[fx2;
    fy2];
n_o=rot*ff1;%定向法向量
ff1_unit=ff1./sum(ff1.^2,1).^0.5;
% 单位法向量，指向曲线内侧
n=diff(ff1_unit,1,2);
n=[n n(:,end)];
flag_dir=sum(n.*n_o,1);

ts=0;route1=r_max;ts_k=1;
for k=2:size(flag_dir,2)-1
    if flag_dir(k)*flag_dir(k+1)<0
       ts=[ts ft(k)];
       ts_k=[ts_k k];
       curvature=norm(cross([ff1(:,k);0],[ff2(:,k);0]))/norm(ff1(:,k))^3;
       route1=[route1 max(curvature,r_max)];
    end
end
ts=[ts ft(end)];
ts_k=[ts_k k+1];
% curvature=norm(cross([ff1(:,end);0],[ff2(:,end);0]))/norm(ff1(:,end))^3;
route1=[route1 r_max];

ts_out=ts;
route_out=route1;
ts_k_out=ts_k;
end